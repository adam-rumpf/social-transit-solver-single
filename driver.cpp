/**
Main driver of the initial objective/constraint evaluation.

Contains the main function, which handles reading in the data, constructing the network, and calling the objective and constraint functions.
*/

#include <ctime>
#include <iomanip>
#include <iostream>
#include "network.hpp"
#include "objective.hpp"
#include "constraints.hpp"

// Define input file names
#define NODE_FILE "data/node_data.txt"
#define ARC_FILE "data/arc_data.txt"
#define OD_FILE "data/od_data.txt"
#define TRANSIT_FILE "data/transit_data.txt"
#define VEHICLE_FILE "data/vehicle_data.txt"
#define OBJECTIVE_FILE "data/objective_data.txt"
#define PROBLEM_FILE "data/problem_data.txt"
#define USER_COST_FILE "data/user_cost_data.txt"
#define OPERATOR_COST_FILE "data/operator_cost_data.txt"
#define ASSIGNMENT_FILE "data/assignment_data.txt"

// Define output file names
#define METRIC_FILE "output/gravity_metrics.txt"
#define SOLUTION_LOG_FILE "output/initial_solution_log.txt"
#define FLOW_FILE "output/initial_flows.txt"

using namespace std;

// Function prototypes
vector<int> read_fleets(string); // reads in initial fleet sizes
void record_metrics(const vector<double> &); // writes accessibility metrics to an output file
void record_flows(Constraint *); // writes flow vector to an output file
void solution_log(const vector<int> &, const vector<double> &); // generates the initial row for the solution log given the solution vector and the element vector
string solution_string(const vector<int> &); // converts a solution vector to a string

/// Main driver.
int main()
{
	clock_t timer; // timer object used to time constraint and objective calculations
	double obj_time = -1; // time required to calculate objective
	double initial_objective = -1; // initial objective value
	double con_time; // time required to calculate constraints
	vector<double> initial_user_costs; // vector of initial user cost components
	vector<double> solution_log_row; // initial row of solution log

	// Get initial solution vector
	vector<int> fleets = read_fleets(TRANSIT_FILE);

	// Initialize network object
	Network * Net = new Network(NODE_FILE, ARC_FILE, OD_FILE, TRANSIT_FILE, VEHICLE_FILE, PROBLEM_FILE);

	// Use Objective object to calculate initial objective value and accessibility metrics
	/*Objective * Obj = new Objective(OBJECTIVE_FILE, Net);
	
	vector<double> metrics = Obj->all_metrics(fleets); // calculate all metrics
	record_metrics(metrics); // write metrics to file
	timer = clock();
	initial_objective = Obj->calculate(fleets); // calculate initial objective value
	obj_time = (1.0*clock() - timer) / CLOCKS_PER_SEC;
	cout << "\nObjective calculation took " << obj_time << " seconds." << endl;
	cout << "Initial objective value: " << initial_objective << endl;*/

	// Use Constraint object to calculate initial constraint function values
	Constraint * Con = new Constraint(USER_COST_FILE, OPERATOR_COST_FILE, ASSIGNMENT_FILE, Net);

	timer = clock();
	initial_user_costs = Con->calculate(fleets).second; // calculate initial user costs
	con_time = (1.0*clock() - timer) / CLOCKS_PER_SEC;
	cout << "\nConstraint calculation took " << con_time << " seconds." << endl;
	cout << "Initial user cost terms: ";
	for (int i = 0; i < initial_user_costs.size(); i++)
		cout << initial_user_costs[i] << ' ';
	cout << endl;

	// Assemble solution log row
	for (int i = 0; i < initial_user_costs.size(); i++)
		solution_log_row.push_back(initial_user_costs[i]);
	solution_log_row.push_back(con_time);
	solution_log_row.push_back(initial_objective);
	solution_log_row.push_back(obj_time);

	// Write solution log row to file
	solution_log(fleets, solution_log_row);

	// Write flows to file
	record_flows(Con);
	
	cin.get();////////////////////////// Remove later.

	return 0;
}

/// Reads in initial fleet sizes from transit data file and returns them as a vector.
vector<int> read_fleets(string transit_file_name)
{
	// Initialize empty fleet size vector
	vector<int> fleets;

	// Read transit data file
	cout << "Reading initial fleet sizes from transit file..." << endl;
	ifstream transit_file;
	transit_file.open(transit_file_name);
	if (transit_file.is_open())
	{
		string line, piece; // whole line and line element being read
		getline(transit_file, line); // skip comment line

		while (transit_file.eof() == false)
		{
			// Get whole line as a string stream
			getline(transit_file, line);
			if (line.size() == 0)
				// Break for blank line at file end
				break;
			stringstream stream(line);

			// Go through each piece of the line
			getline(stream, piece, '\t'); // ID
			getline(stream, piece, '\t'); // Name
			getline(stream, piece, '\t'); // Type
			getline(stream, piece, '\t'); // Fleet
			int new_fleet = stoi(piece);
			getline(stream, piece, '\t'); // Circuit
			getline(stream, piece, '\t'); // Scaling
			getline(stream, piece, '\t'); // LB
			getline(stream, piece, '\t'); // UB
			getline(stream, piece, '\t'); // Fare
			getline(stream, piece, '\t'); // Frequency
			getline(stream, piece, '\t'); // Capacity

			// Create a line object and add it to the list
			fleets.push_back(new_fleet);
		}

		transit_file.close();
	}
	else
		cout << "Transit file failed to load." << endl;

	return fleets;
}

/// Records vector of accessibility metrics to an output file.
void record_metrics(const vector<double> &metrics)
{
	ofstream out_file(METRIC_FILE);

	if (out_file.is_open())
	{
		cout << "Writing metrics to output file..." << endl;

		// Write comment line
		out_file << "Community_Area_ID\tGravity_Metric" << fixed << setprecision(15) << endl;

		// Write all metrics
		for (int i = 0; i < metrics.size(); i++)
			out_file << i + 1 << '\t' << metrics[i] << endl;

		out_file.close();
		cout << "Successfuly recorded metrics!" << endl;
	}
	else
		cout << "Metric file failed to open." << endl;
}

/// Records vector of core arc flows to an output file. Requires a reference to the Constraint object, which contains a flow vector.
void record_flows(Constraint * Con)
{
	ofstream out_file(FLOW_FILE);

	if (out_file.is_open())
	{
		cout << "Writing flows to output file..." << endl;

		// Write comment line
		out_file << "ID\tFlow" << fixed << setprecision(15) << endl;

		// Write all flows
		for (int i = 0; i < Con->Net->core_arcs.size(); i++)
			out_file << Con->Net->core_arcs[i]->id << '\t' << Con->sol_pair.first[i] << endl;

		out_file.close();
		cout << "Successfully recorded flows!" << endl;
	}
	else
		cout << "Flow file failed to open." << endl;
}

/// Records the initial row of the solution log.
void solution_log(const vector<int> &sol, const vector<double> &row)
{
	ofstream log_file(SOLUTION_LOG_FILE);

	if (log_file.is_open())
	{
		cout << "Writing to solution log file..." << endl;

		// Write comment line
		log_file << "Solution\tFeasible\tUC_Riding\tUC_Walking\tUC_Waiting\tCon_Time\tObjective\tObj_Time\t" << fixed << setprecision(15) << endl;

		// Write contents of row vector
		log_file << solution_string(sol) << "\t1\t";
		for (int i = 0; i < row.size(); i++)
			log_file << row[i] << '\t';
		log_file << endl;

		log_file.close();
	}
	else
		cout << "Solution log file failed to open." << endl;

	cout << "Successfully recorded solution!" << endl;
}

/// Converts a solution vector to a string by simply concatenating its digits separated by underscores.
string solution_string(const vector<int> &sol)
{
	string out = "";

	for (int i = 0; i < sol.size(); i++)
		out += to_string(sol[i]) + '_';
	out.pop_back();

	return out;
}
