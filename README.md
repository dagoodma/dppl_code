

## Compiling mex 


The following command worked for me when ran inside the src/ direction:
    `mex -v LDFLAGS='\$LDFLAGS -std=c++11 -g -Wl,-search_paths_first -Wl,-rpath,/Users/dagoodma/asl/dubins_area_coverage/Code/lib/ogdf/_debug -Wl,-rpath,/Users/dagoodma/asl/dubins_area_coverage/Code/lib/LKH -Wl,-headerpad_max_install_names'  -I/Users/dagoodma/asl/dubins_area_coverage/Code/lib/ogdf/include -I/Users/dagoodma/asl/dubins_area_coverage/Code/lib/eigen-eigen-bdd17ee3b1b3 -I/Users/dagoodma/asl/dubins_area_coverage/Code/lib/cxxopts/src -I/Users/dagoodma/asl/dubins_area_coverage/Code/lib/LKH/src/include -I/Users/dagoodma/asl/dubins_area_coverage/Code/include -L/Users/dagoodma/asl/dubins_area_coverage/Code/lib/ogdf/_debug  -L/Users/dagoodma/asl/dubins_area_coverage/Code/lib/LKH -L/Users/dagoodma/asl/dubins_area_coverage/Code/src  -lOGDF -lCOIN -lLKH dubinsAreaCoverage_interface.cpp alternatingDTSP.cpp TSPLib.cpp`

This produced dubinsAreaCoverage_interface.mexmaci64 in the same folder.
