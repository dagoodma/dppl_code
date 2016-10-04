# Dubins Path Planner Library for C++


* Website:  http://dagoodma.github.io/dppl_code
 
* Code: https://github.com/dagoodma/dppl_matlab

* QGC Interface Code: https://github.com/dagoodma/qgroundcontrol/tree/dpp_addon_dtspascpp

* MATLAB Analysis Code: https://github.com/dagoodma/dppl_matlab


The goal of this project was to implement a path planning tool for fixed-wing UAVs, capable of efficiently planning two types of missions. The first is a point-to-point tour over a specific set of points. The second is coverage of an area for collecting data with an onboard sensor. Path length is used as the metric for efficiency in planning paths that conserve the vehicle’s endurance (battery power) allowing it to complete more tasks. This software can plan minimal length paths for accomplishing both types of missions, while also ensuring feasibility by considering the minimum turn radius constraints of the Dubins vehicle.

We have created an open source path planning library called Dubins Path Planning Library (DPPL) to meet the goals outlined above. DPPL was written in C++ as a robust and extensible object-oriented library. Unit testing was implemented through the googletest framework to ensure robustness and aid in cross-platform implementation. DPPL can be used on Mac, Linux, and Windows operating systems. Additional software for plotting and analysis of test data was created for MATLAB (see the MATLAB repo: [DPPL MATLAB](https://github.com/dagoodma/dppl_matlab "dppl_matlab"). Some existing tools and software libraries were utilized in implementing DPPL: the Open Graph Drawing Framework (OGDF) was used for representing graphs and networks, and for reading and writing files in Graph Markup Language (GML) format. Eigen, a C++ template library for linear algebra, was used in some algorithms to simplify computations. For solving ETSP and ATSP tours, we used Helsgaun’s implementation of Lin-Kernighan heuristic, which is a C-based program called LKH. These dependencies will need to be downloaded and compiled separately and placed in the lib/ folder.

## Requirements

DPPL requires a number of libraries. Most of these are included as Git submodules under the `lib/` directory, and can be fetched automatically (see the [Getting Started](#getting-started) section for instructions). Each library will need to be compiled separately before you can compile DPPL. Here's a condensed list of libraries required by DPPL (for a more complete list that includes more information and version numbers, see: [lib/README.md](lib/README.md)):

 * [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
 * [LKH](http://www.akira.ruc.dk/~keld/research/LKH/)
 * [cxxopts](https://github.com/jarro2783/cxxopts)
 * [OGDF](http://ogdf.net/doku.php)
 * [dubins-curves](https://github.com/dagoodma/Dubins-Curves/tree/cpp)
 * [stacktrace.h](lib/stacktrack)

Here is a list of toolchain requirements (mostly imposed by OGDF):

 * CMake 3.1+
 * C++11 compliant compiler
   * gcc 4.8+
   * clang 3.5+
   * Microsoft Visual C++ 2015+
 * GNU Make (in most cases)

## Getting Started

This section explains how to obtain everything needed to use DPPL.

First, clone this Git repo recursively to download all of the submodules (there's a lot of dependencies that are grabbed this way--see the [.gitmodules](.gitmodules) file for the list of submodules, and [lib/README.md](lib/README.md) for additional information about dependencies):

    git clone --recursive https://github.com/dagoodma/dppl_code.git

If you've already cloned this repo but you still need to download the submodules (for example if your `matlab/` folder is empty), then you should run the following commands in your local working copy:

    git submodule init
    git submodule update --recursive —remote

Once you have downloaded DPPL and all of the submodules, you will still need to manually download and compile [LKH 2.0.7](http://www.akira.ruc.dk/~keld/research/LKH/LKH-2.0.7.tgz) and [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)--a header-only library that should be available through your system's package manager (on Mac OS X, we recommend [Homebrew](http://brew.sh/index.html): `brew install eigen`). Extract and compile LKH into `lib/LKH` or `lib/LKH-2.0.7`. You will also need to compile OGDF and Dubins-Curves (requires [SCons](http://scons.org/), which should be available through your system's package manager). 

Now that you have DPPL and all required dependencies, you are ready to compile DPPL itself. We use [CMake](https://cmake.org/) as our build system. Thus you will want to create a new folder (we'll create a `build/` folder inside our local working repo), change into that folder, run `cmake <path-to-dppl>`, and `make`:

    mkdir build
    cd build
    cmake ../
    make

## QGroundControl Interface

An interface was designed in the open source ground station software called QGroundControl (QGC). The interface allows users of QGC to plan efficient waypoint and coverage missions in real time on a map, and then send them to a vehicle controlled by an autopilot. QGC was used as the ground station when experimentally verifying DPPL using a fixed-wing UAV. Find the branch here: [DPPL QGC Interface](https://github.com/dagoodma/qgroundcontrol/tree/dpp_addon_dtspascpp "dpp_addon_dtspascpp (Branch)").


## License

DPPL is licensed under the terms of the MIT license. See LICENSE for more details.

## Author

David Goodman (dagoodma at geemail dot com)

