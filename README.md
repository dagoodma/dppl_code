# Dubins Path Planner Library for C++


* Website:  http://dagoodma.github.io/dppl_code
 
* Code: https://github.com/dagoodma/dppl_matlab

* QGC Interface Code: https://github.com/dagoodma/qgroundcontrol/tree/dpp_addon_dtspascpp

* MATLAB Analysis Code: https://github.com/dagoodma/dppl_matlab


The goal of this project was to implement a path planning tool for fixed-wing UAVs, capable of efficiently planning two types of missions. The first is a point-to-point tour over a specific set of points. The second is coverage of an area for collecting data with an onboard sensor. Path length is used as the metric for efficiency in planning paths that conserve the vehicle’s endurance (battery power) allowing it to complete more tasks. This software can plan minimal length paths for accomplishing both types of missions, while also ensuring feasibility by considering the minimum turn radius constraints of the Dubins vehicle.

We have created an open source path planning library called Dubins Path Planning Library (DPPL) to meet the goals outlined above. DPPL was written in C++ as a robust and extensible object-oriented library. Unit testing was implemented through the googletest framework to ensure robustness and aid in cross-platform implementation. DPPL can be used on Mac, Linux, and Windows operating systems. Additional software for plotting and analysis of test data was created for MATLAB (see the MATLAB repo: [DPPL MATLAB](https://github.com/dagoodma/dppl_matlab "dppl_matlab"). Some existing tools and software libraries were utilized in implementing DPPL: the Open Graph Drawing Framework (OGDF) was used for representing graphs and networks, and for reading and writing files in Graph Markup Language (GML) format. Eigen, a C++ template library for linear algebra, was used in some algorithms to simplify computations. For solving ETSP and ATSP tours, we used Helsgaun’s implementation of Lin-Kernighan heuristic, which is a C-based program called LKH. These dependencies will need to be downloaded and compiled separately and placed in the lib/ folder.

## QGroundControl Interface

An interface was designed in the open source ground station software called QGroundControl (QGC). The interface allows users of QGC to plan efficient waypoint and coverage missions in real time on a map, and then send them to a vehicle controlled by an autopilot. QGC was used as the ground station when experimentally verifying DPPL using a fixed-wing UAV. Find the branch here: [DPPL QGC Interface](https://github.com/dagoodma/qgroundcontrol/tree/dpp_addon_dtspascpp "dpp_addon_dtspascpp (Branch)").

## License

DPPL is licensed under the terms of the MIT license. See LICENSE for more details.

## Author

David Goodman (dagoodma at geemail dot com)

