#include "assignment.h"

/// Constant-cost assignment constructor sets network pointer.
ConstantAssignment::ConstantAssignment(Network * net_in)
{
	Net = net_in;
	stop_size = Net->stop_nodes.size();
}

/**
Constant-cost assignment model evaluation for a given solution.

Requires a fleet size vector.

Returns a pair containing a vector of flow values and a waiting time scalar.

This model comes from the linear program formulation of the common line problem, which can be solved using a Dijkstra-like label setting algorithm. This must be done separately for every sink node, but each of these problems is independent and may be parallelized. The final result is the sum of these individual results.
*/
pair<vector<double>, double> ConstantAssignment::calculate(vector<int> &fleet)
{
	// Generate a vector of line frequencies based on the fleet sizes
	vector<double> line_freq(Net->lines.size());
	for (int i = 0; i < line_freq.size(); i++)
		line_freq[i] = Net->lines[i]->frequency(fleet[i]);

	// Use the line frequencies to generate arc frequencies
	vector<double> freq(Net->core_arcs.size(), INFINITY);
	for (int i = 0; i < Net->lines.size(); i++)
		for (int j = 0; j < Net->lines[i]->boarding.size(); j++)
			freq[Net->lines[i]->boarding[j]->id] = line_freq[i];

	// Solve single-destination model in parallel for all sinks and add all results
	vector<double> flows(Net->core_arcs.size(), 0.0); // total flow vector over all destinations
	double wait = 0.0; // total waiting time over all destinations
	cout << "Solving single-sink models in parallel:\n|";
	for (int i = 0; i < stop_size; i++)
		cout << '-'; // length of "progress bar"
	cout << "|\n|";
	parallel_for_each(Net->stop_nodes.begin(), Net->stop_nodes.end(), [&](Node * s)
	{
		cout << '*'; // shows progress
		flows_to_destination(s->id, flows, wait, freq, flow_lock);
	});
	cout << '|' << endl;





	///////////////////////////////////////////////// all file output below is temporary for debugging
	ofstream flow_file("output/flows.txt");
	if (flow_file.is_open())
	{
		cout << "Writing flow file..." << endl;

		// Write waiting time
		flow_file << "Waiting: " << wait << endl;

		// Write comment line
		flow_file << "ArcID\tFlow" << endl;

		// Write contents of flow vector
		for (int i = 0; i < flows.size(); i++)
			flow_file << i << '\t' << flows[i] << endl;
	}
	else
		cout << "Solution log file failed to open." << endl;
	cout << "Successfully recorded solution!" << endl;
	//////////////////////////////////////////////

	return make_pair(flows, wait);
}

/**
Calculates the flow vector to a given sink.

Requires the sink index (as a position in the stop node list), flow vector, waiting time scalar, and line frequency vector.

The flow vector and waiting time are passed by reference and automatically incremented according to the results of this function.

The algorithm here solves the constant-cost, single-destination version of the common lines problem, which is a LP similar to min-cost flow and is solvable with a Dijkstra-like label setting algorithm. This process can be parallelized over all destinations, and so should rely only on local variables.
*/
void ConstantAssignment::flows_to_destination(int dest, vector<double> &flows, double &wait, vector<double> &freq, reader_writer_lock &flow_lock)
{
	/*
	To explain a few technical details, the label setting algorithm involves updating a distance label for each node. In each iteration, we choose the unprocessed arc with the minimum value of its own cost plus its head's label. In order to speed up that search, we store all of those values in a min-priority queue. As with Dijkstra's algorithm, to get around the inability to update priorities, we just add extra copies to the queue whenever they are updated. We also store a master list of those values, which should always decrease as the algorithm moves forward, as a comparison every time we pop something out of the queue to ensure that we have the latest version.

	The arc loading algorithm involves processing all of the selected attractive arcs in descending order of their cost-plus-head-label from the label setting algorithm. This is accomplished in a similar way, with a copy of the cost-plus-head-label being added to a max-priority queue each time the tail label is updated.
	*/

	// Initialize data structures
	vector<double> node_label(Net->core_nodes.size(), INFINITY); // tentative distances from every node to the destination
	node_label[Net->stop_nodes[dest]->id] = 0.0; // distance from destination to self is 0
	vector<double> node_freq(Net->core_nodes.size(), 0.0); // total frequency of all attractive arcs leaving a node
	vector<double> node_vol(Net->core_nodes.size(), 0.0); // total flow leaving a node
	for (int i = 0; i < Net->stop_nodes.size(); i++)
		// Initialize travel volumes for stop nodes based on demand for destination
		node_vol[Net->stop_nodes[i]->id] = Net->stop_nodes[dest]->incoming_demand[i];
	unordered_set<int> unprocessed_arcs; // arcs not yet chosen in the main label setting loop, in order to ensure that each arc is processed only once
	for (int i = 0; i < Net->core_arcs.size(); i++)
		// All arcs are initially unprocessed
		unprocessed_arcs.insert(Net->core_arcs[i]->id);
	priority_queue<arc_cost_pair, vector<arc_cost_pair>, greater<arc_cost_pair>> arc_queue; // min-priority queue to quickly access the unprocessed arc with the minimum cost-plus-head-distance value
	for (int i = 0; i < Net->stop_nodes[dest]->core_in.size(); i++)
		// Set all non-infinite arc labels (which will include only the sink node's incoming arcs)
		arc_queue.push(make_pair(Net->stop_nodes[dest]->core_in[i]->cost, Net->stop_nodes[dest]->core_in[i]->id));
	unordered_set<int> attractive_arcs; // set of attractive arcs
	priority_queue<arc_cost_pair, vector<arc_cost_pair>, less<arc_cost_pair>> load_queue; // max-priority queue to process attractive arcs in reverse order

	// Main label setting loop

	// Initialize label setting variables
	double min_label; // chosen arc's cost-plus-head-label value
	int min_arc; // chosen node's incoming arc
	int min_tail; // chosen arc's tail node ID
	int updated_arc; // ID of arc whose queue position must be updated
	double updated_label; // arc queue key for update

	while (unprocessed_arcs.empty() == false && arc_queue.empty() == false)
	{
		// Find the arc that minimizes the sum of its head's label and its own cost
		min_label = arc_queue.top().first;
		min_arc = arc_queue.top().second;
		arc_queue.pop();

		// Only proceed for unprocessed arcs
		if (unprocessed_arcs.count(min_arc) == 0)
			continue;

		// Mark arc as processed and get its tail
		unprocessed_arcs.erase(min_arc);
		min_tail = Net->core_arcs[min_arc]->tail->id;

		// Skip arcs with zero frequency (can occur for boarding arcs on lines with no vehicles)
		if (freq[min_arc] == 0)
			continue;

		// Update the node label of the chosen arc's tail
		if (node_label[min_tail] >= min_label)
		{
			// Check whether the attractive arc has infinite frequency
			if (freq[min_arc] < INFINITY)
			{
				// Finite-frequency attractive arc (should include only boarding arcs)

				// Update tail label
				if (node_label[min_tail] < INFINITY)
					// Standard update
					node_label[min_tail] = (node_freq[min_tail] * node_label[min_tail] + freq[min_arc] * min_label) / (node_freq[min_tail] + freq[min_arc]);
				else
					// First-time update (from initially-infinite label)
					node_label[min_tail] = (1 / freq[min_arc]) + min_label;

				// Update tail frequency
				node_freq[min_tail] += freq[min_arc];
			}
			else
			{
				// Infinite-frequency attractive arc

				// Update tail label and frequency
				node_label[min_tail] = min_label;
				node_freq[min_tail] = INFINITY;

				// Remove all other attractive arcs leaving the tail
				for (int i = 0; i < Net->core_nodes[min_tail]->core_out.size(); i++)
					attractive_arcs.erase(Net->core_nodes[min_tail]->core_out[i]->id);
			}

			// Add arc to attractive arc set
			attractive_arcs.insert(min_arc);

			// Update arc labels that are affected by the updated tail node
			for (int i = 0; i < Net->core_nodes[min_tail]->core_in.size(); i++)
			{
				// Find arcs to update, recalculate labels, and push updates into priority queue
				updated_arc = Net->core_nodes[min_tail]->core_in[i]->id;
				updated_label = Net->core_arcs[updated_arc]->cost + node_label[min_tail];
				arc_queue.push(make_pair(updated_label, updated_arc));
			}
		}
	}

	// Build updated max-priority queue for attractive arc set

	/////////////////////////////////////////////// go through every arc; skip if not chosen; if boarding, also add this headway*vol to the waiting time total

	for (auto a = attractive_arcs.begin(); a != attractive_arcs.end(); a++)
	{
		// Recalculate the cost-plus-head label for each attractive arc and place in a max-priority queue
		load_queue.push(make_pair(node_label[Net->core_arcs[*a]->head->id] + Net->core_arcs[*a]->cost, *a));
	}

	// Main arc loading loop

	// Initialize loading loop variables
	int max_arc; // chosen arc ID
	int max_head; // chosen arc's head node ID
	int max_tail; // chosen arc's tail node ID
	double added_flow; // chosen arc's added flow volume

	while (load_queue.empty() == false)
	{
		// Process attractive arcs in descending order of cost-plus-head-label value

		// Get next arc's properties
		max_arc = load_queue.top().second;
		max_tail = Net->core_arcs[max_arc]->tail->id;
		max_head = Net->core_arcs[max_arc]->head->id;

		// Distribute volume from tail
		if (freq[max_arc] < INFINITY)
			// Finite-frequency arc
			added_flow = (freq[max_arc] / node_freq[max_tail]) * node_vol[max_tail];
		else
			// Infinite-frequency arc
			added_flow = node_vol[max_tail];

		// Update head
		node_vol[max_head] += added_flow;

		// Update total destination-independent arc volume
		////////////////////////////////// Redo this to maintain a queue of pairs of (id,increase) values that includes only the nonzero changes, then lock down each thread while it processes its own arc queue.
		flow_lock.lock();
		flows[max_arc] += added_flow;
		flow_lock.unlock();

		// Remove arc from priority queue
		load_queue.pop();
	}

	///////////////////////////////////////// calculate waiting time; for each i it should be the min of all v_a/f_a for every arc a that leaves node i; shouldn't have to worry about zero frequency arcs since they can never be in the attractive set
	/*
	Waiting time w_i is meant to satisfy v_a <= f_a w_i for all arcs a that leave node i.
	If we know the volumes then we have (v_a/f_a) <= w_i, and we want to minimize w_i, so w_i should equal the maximum v_a/f_a over all leaving arcs.
	We could do this by processing each node one-by-one at the end, or by updating the node waiting times as we load the arcs, where at each step we decide whether or not to increase the tail node's waiting time.
	If the arc is infinite-frequency, then its tail will never have any waiting time, so during infinite-frequency arc loading we don't need to do anything.
	*/
}

/// Nonlinear assignment constructor reads in model data from file and sets network pointer.
NonlinearAssignment::NonlinearAssignment(string input_file, Network * net_in)
{
	Net = net_in;

	// Initialize submodel object
	Submodel = new ConstantAssignment(net_in);

	// Read assignment model data
	cout << "Reading assignment model data..." << endl;
	ifstream a_file;
	a_file.open(input_file);
	if (a_file.is_open())
	{
		string line, piece; // whole line and line element being read
		getline(a_file, line); // skip comment line
		int count = 0;

		while (a_file.eof() == false)
		{
			count++;

			// Get whole line as a string stream
			getline(a_file, line);
			if (line.size() == 0)
				// Break for blank line at file end
				break;
			stringstream stream(line);

			// Go through each piece of the line
			getline(stream, piece, '\t'); // Label
			getline(stream, piece, '\t'); // Value
			string value = piece;

			// Expected data
			if (count == 1)
				error_tol = stod(value);
			if (count == 2)
				max_iterations = stoi(value);
			if (count == 4)
				conical_alpha = stod(value);
			if (count == 5)
				conical_beta = stod(value);
		}

		a_file.close();
	}
	else
		cout << "Assignment file file failed to open." << endl;

	cout << "Assignment data read." << endl;
}

/**
Nonlinear cost assignment model evaluation for a given solution.

Requires a fleet size vector and an initial solution, which takes the form of a pair made up of a flow vector and a waiting time scalar.

Returns a pair containing a vector of flow values and a waiting time scalar.

The solution vector is used to determine the frequency and capacity of each line. Frequencies contribute to boarding arc costs for the common lines problem, while capacities contribute to an overcrowding penalty to model congestion.

The overall process being used here is the Frank-Wolfe algorithm, which iteratively solves the linear approximation of the nonlinear cost quadratic program. That linear approximation happens to be an instance of the constant-cost LP whose costs are based on the current solution.
*/
pair<vector<double>, double> NonlinearAssignment::calculate(vector<int> &fleet, pair<vector<double>, double> initial_sol)
{
	// Get initial solution
	vector<double> flows = initial_sol.first;
	double wait = initial_sol.second;

	////////////////////////////////////////////////////////////////
	/////////////////////// For now, just directly call the nonlinear model.
	////////////////////////////////////////////////////////////////
	make_pair(flows, wait) = Submodel->calculate(fleet);








	////////////////////////////
	return make_pair(flows, wait);
}
