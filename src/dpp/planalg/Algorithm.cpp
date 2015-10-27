/*
 * Implementation of DPP::Algorithm abstract class.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdexcept>
#include <stdio.h>

#include <dpp/planalg/Algorithm.h>

// For unix we use popen, TODO remove this
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))

//#include <unistd.h>

#endif


namespace dpp {

const char* Algorithm::TypeText[] = { "Dtsp", "CPP" };

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
    std::string shell_command = LKH_EXECUTABLE " " + parFilename;

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    #include <errno.h>
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Running LKH Solver with popen(): "
        << const_cast<char*>(shell_command.c_str()) << std::endl;
    FILE *pFile = popen(const_cast<char*>(shell_command.c_str()), "r");
    // Read through stdout until it ends
    // TODO add error checking/parsing here
    char buffer [100];
    while ( ! feof (pFile) )
     {
       if ( fgets (buffer , 100 , pFile) == NULL ) break;
       //fputs (buffer , stdout);
     }

    result = pclose(pFile);
    // Additional error reporting in Unix
    if (result != SUCCESS) {
        int errsv = errno;
        Logger::logError() << "Failed with error code: " << errsv << std::endl;
    }
#else
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Running LKH Solver with system(): "
        const_cast<char*>(cmd.c_str()) << std::endl;
    // If we don't have open, stdout is printed to terminal
    result = system(const_cast<char*>(cmd.c_str()));
#endif

    if (result != SUCCESS) {
        throw std::runtime_error(string("LKH solver failed with code: ") + to_string(result));
    }
  
    return result;
}

} // namespace dpp