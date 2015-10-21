# Project Dependencies

## Library Dependencies

The following libraries are used by this software:

* ogdf [Baobab 2015-06-29.snapshot]
    http://ogdf.net/doku.php
    https://github.com/ogdf/ogdf

* Eigen3
    http://eigen.tuxfamily.org/index.php?title=Main_Page

* cxxopts [2015-09-27]
    https://github.com/jarro2783/cxxopts

* dubins-curves [2014-03-22]
  Note: This library was modified and compiled into a static c++ library.

    https://github.com/AndrewWalker/Dubins-Curves

* stacktrace.h [2008]
  Note: This is used only in the project binaries for debugging purposes.
  TODO: This does not demangle symbols on Mac OS X. For that, see here: 
  https://github.com/MesserLab/SLiM/search?utf8=%E2%9C%93&q=eidos_print_stacktrace

    https://panthema.net/2008/0901-stacktrace-demangled/


## Binary Dependencies

The following binaries are used by this software:

* LKH 2.0.7
  Note: LKH must be compiled manually, and the binary placed in the build directory.
