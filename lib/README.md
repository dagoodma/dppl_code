# Project Dependencies

The following dependencies used by this software are not provided as submodules, and 
should be installed with a package manager or manually (LKH):

* Eigen3
    - http://eigen.tuxfamily.org/index.php?title=Main_Page

* LKH 2.0.7
    - http://www.akira.ruc.dk/~keld/research/LKH/
    - http://www.akira.ruc.dk/~keld/research/LKH/LKH-2.0.7.tgz
    - *NOTE*: LKH must be compiled manually, and the binary placed in your DPPL/cmake build directory.

The rest of the dependecies can be obtained by cloning this project recursively
with `git clone --recursive <url>`, or checking out after cloning with
`git submodule update --init --recursive`. They are as follows:

* cxxopts [2015-09-27]
    - https://github.com/jarro2783/cxxopts

* OGDF [Baobab 2015-06-29.snapshot]
    - http://ogdf.net/doku.php
    - https://github.com/ogdf/ogdf

* dubins-curves [2014-03-22]
    - https://github.com/dagoodma/Dubins-Curves/tree/cpp, C++ fork of:
    - https://github.com/AndrewWalker/Dubins-Curves
    - *NOTE*: This library was modified and compiled into a static C++ library.

* stacktrace.h [2008]
    - http://panthema.net/2008/0901-stacktrace-demangled/
    - *NOTE*: This is already included with the DPPL code, and it is only used in the
        project binaries for debugging purposes.
    - *TODO*: This does not demangle symbols on Mac OS X. For that, see here: 
      https://github.com/MesserLab/SLiM/search?utf8=%E2%9C%93&q=eidos_print_stacktrace
