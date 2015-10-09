/*
 * Implementation of DPP::Algorithm abstract class.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdexcept>

#include <dpp/planalg/Algorithm.h>

namespace dpp {

const char* Algorithm::TypeText[] = { "DTSP", "CPP" };


Algorithm::~Algorithm() {
    
}


/**
 * Makes a system call to run the LKH solver.
 * @throws a runtime exception if the solver fails.
 * @return SUCCESS or FAILURE
 * @TODO create a new thread, capture output
 */
int Algorithm::runLKHSolver(std::string parFilename) {
    int result = FAILURE;
    std::string cmd = "LKH ";
    cmd += parFilename;
    result = system(const_cast<char*>(cmd.c_str()));

    if (result != SUCCESS) {
        throw std::runtime_error(string("LKH solver failed with code: ") + to_string(result));
    }
  
    return result;
}

} // namespace dpp