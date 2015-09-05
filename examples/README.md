# Example Code

To build the example code, create a build directory and run cmake:

    mkdir build
    cd build
    cmake ..

To build in debug mode, run:

    mkdir build-debug
    cd build-debug
    cmake .. -DCMAKE_BUILD_TYPE=Debug

Examples output dot files to std::out. The can be viewed in graph-viz through the gui. You can also use the `dot` command to convert the files into images for viewing. Try this:

    ./dgaTest > test.dot && dot -Tpng test.dot > test.png && open test.png


For gml files, try converting them to dot files:

    gml2gv -otest.dot test.gml
