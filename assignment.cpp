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
	vector<double> freq(Net->core_arcs.size(), FINITE_INFINITY);
	for (int i = 0; i < Net->lines.size(); i++)
		for (int j = 0; j < Net->lines[i]->boarding.size(); j++)
			freq[Net->lines[i]->boarding[j]->id] = line_freq[i];

	///////////////////////////////////////////////////////////////// write serialized version for now, and convert to parallel later (may need map/reduce; may just need a concurrent vector)
	// Solve single-destination model for all sinks and add all results
	vector<double> flows(Net->core_arcs.size(), 0.0); // total flow vector over all destinations
	double wait = 0.0; // total waiting time over all destinations
	cout << "Solving single-sink models for sink: ";
	for (int i = 0; i < stop_size; i++)
	{
		cout << i << ' ';
		flows_to_destination(i, flows, wait, freq);


		break;/////////////////////////////////////////////////
	}
	cout << endl;





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
void ConstantAssignment::flows_to_destination(int dest, vector<double> &flows, double &wait, vector<double> &freq)
{
	/*
	To explain a few technical details, the label setting algorithm involves updating a distance label for each node. In each iteration, we choose the unprocessed arc with the minimum value of its own cost plus its head's label. In order to speed up that search, we store all of those values in a min-priority queue. As with Dijkstra's algorithm, to get around the inability to update priorities, we just add extra copies to the queue whenever they are updated. We also store a master list of those values, which should always decrease as the algorithm moves forward, as a comparison every time we pop something out of the queue to ensure that we have the latest version.
	*/

	// Initialize label setting structures
	vector<double> node_label(Net->core_nodes.size(), FINITE_INFINITY); // tentative distances from every node to the destination
	node_label[Net->stop_nodes[dest]->id] = 0.0; // distance from destination to self is 0
	vector<double> node_freq(Net->core_nodes.size(), 0.0); // total frequency of all attractive lines leaving a node
	vector<double> node_vol(Net->core_nodes.size(), 0.0); // total flow leaving a node
	unordered_set<int> unprocessed_arcs; // arcs not yet chosen in the main label setting loop, in order to ensure that each arc is processed only once
	for (int i = 0; i < Net->core_arcs.size(); i++)
		unprocessed_arcs.insert(Net->core_arcs[i]->id); // all arcs are initially unprocessed
	priority_queue<arc_cost_pair, vector<arc_cost_pair>, greater<arc_cost_pair>> arc_queue; // min-priority queue to quickly access the unprocessed arc with the minimum cost-plus-head-distance value
	for (int i = 0; i < Net->stop_nodes[dest]->core_in.size(); i++)
		// Set all non-infinite arc labels (which will include only the sink node's incoming arcs)
		arc_queue.push(make_pair(Net->stop_nodes[dest]->core_in[i]->cost, Net->stop_nodes[dest]->core_in[i]->id));
	stack<int> attractive_arcs; // set of attractive arcs, stored as a stack for loading in reverse order

	int count = 0;////////////////////////////
	cout << "\nProcessing destination node " << dest << endl;
	// Main label setting loop (continues until every arc has been processed)
	while (unprocessed_arcs.empty() == false && arc_queue.empty() == false)
	{
		count++;/////////////////////////////

		// Find the arc that minimizes the sum of its head's label and its own cost
		double min_label = arc_queue.top().first;
		int min_arc = arc_queue.top().second;
		cout << "Picking arc " << min_arc << " (" << Net->core_arcs[min_arc]->tail->id << "," << Net->core_arcs[min_arc]->head->id << ") with u+c = " << min_label << endl;
		arc_queue.pop();

		// Only proceed for unprocessed arcs
		if (unprocessed_arcs.count(min_arc) == 0)
		{
			cout << "Skipping arc " << min_arc << " (already processed)." << endl;
			continue;
		}
		cout << "Keeping arc " << min_arc << endl;

		unprocessed_arcs.erase(min_arc);
		int min_tail = Net->core_arcs[min_arc]->tail->id;

		// Update the node label of the chosen arc's tail
		cout << "Updating node label for a frequency-based arc." << endl;
		if (node_label[min_tail] < FINITE_INFINITY)
		{
			// Standard node label update
			cout << "Updating node label for a finite-frequency arc (standard)." << endl;
			node_label[min_tail] = (node_freq[min_tail] * node_label[min_tail] + freq[min_arc] * min_label) / (node_freq[min_tail] + freq[min_arc]);
			node_freq[min_tail] += freq[min_arc];
			attractive_arcs.push(min_arc);
		}
		else
		{
			// First-time update to deal with the initially-infinite node label
			cout << "Updating node label for a finite-frequency arc (first time)." << endl;
			node_label[min_tail] = (1 / freq[min_arc]) + min_label;
			node_freq[min_tail] = freq[min_arc];
			attractive_arcs.push(min_arc);
		}

		// Update arc labels that are affected by the updated tail node
		for (int i = 0; i < Net->core_nodes[min_tail]->core_in.size(); i++)
		{
			arc_queue.push(make_pair(Net->core_nodes[min_tail]->core_in[i]->cost + node_label[min_tail], Net->core_nodes[min_tail]->core_in[i]->id));
			cout << "Updating arc " << Net->core_nodes[min_tail]->core_in[i]->id << " (" << Net->core_nodes[min_tail]->core_in[i]->tail->id << "," << Net->core_nodes[min_tail]->core_in[i]->head->id << ") with u+c = " << Net->core_nodes[min_tail]->core_in[i]->cost + node_label[min_tail] << endl;
		}








		///////////////////////////////
		if (count > 20)
		{
			cout << "Artificially ending." << endl;
			break;
		}
		cout << "Made it to loop end." << endl;
		cout << "Unprocessed arcs size = " << unprocessed_arcs.size() << endl;
		cout << "Arc queue size = " << arc_queue.size() << endl;
		cout << "-----------------------------" << endl;
		//////////////////////////////////
	}

	cout << "Part 1 ended." << endl;

	// Arc loading
	///////// go through every arc; skip if not chosen; if boarding, also add this headway*vol to the waiting time total
	///////// the process order should come from just popping the stack elements
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
