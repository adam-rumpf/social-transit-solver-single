# social-transit-solver-single

A solution algorithm initialization program for use in a research project of mine dealing with a public transit design model with social access objectives. This produces the initial objective and constraint function values, and is not part of the main solution search loop.

# Data Folder

The main program looks for input files in a local `data/` folder. The following data files should be included in the this folder:

* [`arc_data.txt`](#arc_datatxt)
* [`assignment_data.txt`](#assignment_datatxt)
* [`node_data.txt`](#node_datatxt)
* [`objective_data.txt`](#objective_datatxt)
* [`od_data.txt`](#od_datatxt)
* [`operator_cost_data.txt`](#operator_cost_datatxt)
* [`problem_data.txt`](#problem_datatxt)
* [`transit_data.txt`](#transit_datatxt)
* [`user_cost_data.txt`](#user_cost_datatxt)
* [`vehicle_data.txt`](#vehicle_datatxt)

The contents of these files will be explained below. Most include IDs for each of their elements. For the purposes of our solution algorithm, these are assumed to consecutive integers beginning at `0`, and this is how they will be treated for the purposes of array placement.

Unless otherwise specified, the following units are used:

* cost = dollars
* distance = miles
* time = minutes

## `arc_data.txt`

Information related to all arcs.

Contains the following columns:
* `ID`: Unique identifying number.
* `Type`: Arc type ID. The types in use are:
  * `0`: line arc
  * `1`: boarding arc
  * `2`: alighting arc
  * `3`: core network walking arc (stop nodes only)
  * `4`: accessibility network walking arc (stop nodes, population centers, and facilities)
* `Line`: Line ID of a line arc, and `-1` otherwise.
* `Tail`: Node ID of the arc's tail.
* `Head`: Node ID of the arc's head.
* `Time`: Constant part of travel time of arc. Boarding arcs, whose travel time is based on the line frequency, have a listed time of `0`.

## `assignment_data.txt`

Information related to the Spiess and Florian assignment model.

Contains the following rows:
* `Epsilon`: Optimality gap threshold to use for ending the Frank-Wolfe algorithm.
* `Cutoff`: Iteration cutoff for the Frank-Wolfe algorithm.
* `Elements`: Number of parameters listed on the following rows. Currently set to `2`.
* `alpha`: Alpha parameter of the conical congestion function.
* `beta`: Beta parameter of the conical congestion function. By definition it should equal `(2 alpha - 1)/(2 alpha - 2)`, and is included here only for convenience.

## `node_data.txt`

Information related to all nodes.

Contains the following columns:
* `ID`: Unique identifying number. Used to reference specific nodes in the other data files.
* `Name`: Name of the node. Most stops are simply called "Stop" followed by their ID number. Boarding nodes also append the name of their line. Population centers and primary care facilities use their real names.
* `Type`: Node type ID. The types in use are:
  * `0`: stop node
  * `1`: boarding node
  * `2`: population center
  * `3`: primary care facility
* `Line`: Line ID of a boarding node, and `-1` otherwise.
* `Value`: Population of a population center, facility weight of a primary care facility, and `-1` otherwise.

## `objective_data.txt`

Information related to defining the objective function and related accessibility metrics.

Contains the following rows:
* `Elements`: Number of parameters listed on the following rows. Currently set to `4`.
* `Lowest`: Number of lowest-metric population centers to take for the objective function.
* `Gravity_Falloff`: Exponent used to define distance falloff in gravity metric. This should be a positive value, and will be treated as negative in the program. A larger value means faster falloff.
* `Multiplier`: Factor by which to multiply the objective value in the solution log. This should be chosen to compensate for very small decimal values that would otherwise risk truncation error.

## `od_data.txt`

Origin/destination travel demands. Only nonzero demands are meant to be included.

Contains the following columns:
* `ID`: Unique identifying number.
* `Origin`: Node ID of origin.
* `Destination`: Node ID of destination.
* `Volume`: Number of people wishing to travel from the origin to the destination.

## `operator_cost_data.txt`

Information related to defining the operator cost function.

Contains the following rows:
* `Initial`: Operator cost of the initial solution. Used for defining the allowable relative increase bounds. Note that this is not calculated automatically during preprocessing, and must be filled in by hand by using the single-run model.
* `Elements`: Number of parameters listed on the following rows. Currently set to `2`.
* `Operating_Cost`: Weight of the vehicle operating costs.
* `Fares`: Fare collected from each boarding.

## `problem_data.txt`

Miscellaneous data required to define the problem.

Contains the following rows:
* `Elements`: Number of parameters listed on the following rows. Currently set to `1`.
* `Horizon`: Total daily time horizon.

## `transit_data.txt`

Information related to each transit line. This includes a variety of fields directly related to the solution vector, such as the initial number of vehicles on each line (which constitutes the initial solution vector).

Contains the following columns:
* ID: Unique identifying number. This should indicate its position in the solution vector. Also used for line-specific references in the other data files.
* `Name`: Name of the line listed in the GTFS files.
* `Type`: Vehicle type ID. Matches one of the IDs listed in the vehicle data file.
* `Fleet`: Initial fleet size.
* `Circuit`: Total time for a vehicle to complete one circuit.
* `Scaling`: Fraction of the day during which the line is active. `1.0` indicates the entire daily time horizon.
* `LB`: Lower bound of allowable fleet size.
* `UB`: Upper bound of allowable fleet size.
* `Fare`: Boarding fare.
* `Frequency`: Initial line frequency, measured only during its active portion of the day. Relatively unimportant since it is recalculated each iteration for the current solution.
* `Capacity`: Initial line capacity, measured as a total number of passengers that can be transported per day. Relatively unimportant since it is recalculated each iteration for the current solution.

## `user_cost_data.txt`

Information related to defining the user cost function.

Contains the following rows:
* `Initial`: User cost of the initial solution. Used for defining the allowable relative increase bounds. Note that this is not calculated automatically during preprocessing, and must be filled in by hand by using the single-run model.
* `Elements`: Number of parameters listed on the following rows. Currently set to `3`.
* `Riding`: Weight of in-vehicle riding time.
* `Walking`: Weight of walking time.
* `Waiting`: Weight of waiting time.

## `vehicle_data.txt`

Information related to each vehicle. Each route has a specified vehicle type, and only routes with the same vehicle type may exchange vehicles during the search algorithm.

Contains the following columns:
* `Type`: Vehicle type ID. Referenced in the transit data file to specify the vehicle type of each route.
* `Name`: Name of the vehicle.
* `UB`: Maximum number of this type of vehicle allowed in the network.
* `Capacity`: Seating capacity of this vehicle type. It is assumed that loading factor has already been taken into account. For trains, it is assumed that this includes all cars on a given train.
* `Cost`: Operating cost per unit time.
