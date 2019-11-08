/**
Implementation of the nonlinear-cost Spiess and Florian user assignment model.

Solves the Spiess and Florian model to return the user flows based on a given solution. This involves conducting the Frank-Wolfe algorithm on the nonlinear model, which in turn involves iteratively solving the constant-cost version of the model.
*/

#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <stack>
#include <string>
#include <ppl.h>
#include <queue>
#include <utility>
#include <unordered_set>
#include <vector>
#include "network.hpp"

using namespace std;
using namespace concurrency;

typedef pair<double, int> arc_cost_pair; // used to define a priority queue of combined cost/ID pairs sorted by the first element

/**
Constant-cost assignment model class.

Includes a variety of attributes and methods for evaluating the constant-cost version of the Spiess and Florian model.

This could technically be used as an assignment model all on its own, but its main purpose is as a subroutine within the nonlinear model, which involves iteratively solving and re-solving the constant-cost model.
*/
struct ConstantAssignment
{
	// Public attributes
	Network * Net; // pointer to network object
	int stop_size; // number of stop nodes in network

	// Public methods
	ConstantAssignment(Network *); // constructor sets network pointer
	pair<vector<double>, double> calculate(vector<int> &); // calculates flow vector for a given fleet vector
	void flows_to_destination(int, vector<double> &, double &, vector<double> &, reader_writer_lock &, reader_writer_lock &); // calculates flow vector and waiting time for a single given sink
};

/**
Nonlinear cost assignment model cost.

Includes a variety of attributes and methods for evaluating the nonlinear cost version of the Spiess and Florian model.

This model is evaluated by conducting the Frank-Wolfe algorithm on a nonlinear program. Each iteration requires solving the constant-cost version. The process halts either after an optimality bound cutoff or an iteration cutoff.
*/
struct NonlinearAssignment
{
	// Public attributes
	Network * Net; // pointer to network object
	ConstantAssignment * Submodel; // pointer to constant-cost submodel
	double error_tol; // error bound cutoff for Frank-Wolfe
	int max_iterations; // iteration cutoff for Frank-Wolfe
	double root_error_tol; // function value cutoff for root finding
	int root_max_iterations; // iteration cutoff for root finding
	double conical_alpha; // alpha parameter for conical congestion function
	double conical_beta; // beta parameter for conical congestion function

	// Public methods
	NonlinearAssignment(string, Network *); // constructor reads assignment model data file and sets network pointer
	pair<vector<double>, double> calculate(vector<int> &, pair<vector<double>, double>); // calculates flow vector for a given fleet vector and initial assignment model solution
	double arc_cost(int, double, double); // calculates the nonlinear cost function for a given arc
	double arc_cost_prime(int, double, double); // first derivative of arc cost function
	double obj_prime(double, vector<double> &, vector<double> &, double &, vector<double> &, double &); // convex combination of nonlinear objective's previous and next solutions
	double obj_prime_2(double, vector<double> &, vector<double> &, double &, vector<double> &, double &); // derivative of convex combination of nonlinear objective's previous and next solutions
};
