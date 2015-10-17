/*
 * MEX interface for the solveDtsp binary.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <string>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>

#include "mexHelper.h"
#include "solveDtsp.h"

#define LOG_FILENAME "solveDtsp.log"

/*
 * MEX gateway function. The MEX file synopsis is as follows:
 *
 *      solveDtsp V x r [return] [algorithm]
 *
 * Parameters:
 *      V   Vertices, n-by-2 matrix of node positions
 *      x   Initial heading [rad]
 *      r   Vehicle turn radius [m]
 *      [return]      Optional argument whether to return to the starting node
 *      [algorithm]   Optional algorithm name. Options are "alternating",
 *                    "nearest", and "randomized".
 * Returns:
 *      E       Edges of tour, m-by-3 matrix with rows as: srcNodeId, targNodeId, cost
 *      X       Headings of tour, n-by-1 vector of node headings
 *      cost    Total cost of tour
 */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    // Enable logging
    #ifdef MEX_DEBUG
    dpp::Logger *log = dpp::Logger::Instance();
    log->level(dpp::Logger::Level::LL_DEBUG);
    log->verbose(2);
    dpp::Logger::initializeLogger(LOG_FILENAME);
    #endif

    /* check for proper number of arguments */
    if(nrhs<3) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nrhs",
                      "At least three inputs are required.");
    }
    if(nrhs>5) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nrhs",
                      "Too many input arguments given.");
    }

    if(nlhs!=3) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nlhs",
                      "Three outputs required.");
    }

    // check that columns in V is 2 
    if(mxGetN(prhs[0])!=2) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":V:wrongColumns",
                          "V must have 2 columns.");
    }

    // check that rows in V is larger than 1
    if(mxGetM(prhs[0])<2) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":V:notEnoughRows",
                          "V must have at least 2 rows.");
    }

    // Ensure x and r are scalar Doubles
    if( !mxIsDouble(prhs[1]) || 
         mxIsComplex(prhs[1]) ||
         mxGetNumberOfElements(prhs[1])!=1 ) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":x:notScalar",
                          "Input heading must be a scalar.");
    }

    if( !mxIsDouble(prhs[2]) || 
         mxIsComplex(prhs[2]) ||
         mxGetNumberOfElements(prhs[2])!=1 ) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":r:notScalar",
                          "Input radius must be a scalar.");
    }

    size_t n = mxGetM(prhs[0]);
    size_t cols = mxGetN(prhs[0]);

    // Input variables
    double *V, x, r;             // input variables
    bool returnToInitial = false;
    std::string algorithmName("alternating");

    // Output variables
    double *E, *X, cost;         // output variables

    V = mxGetPr(prhs[0]);
    x = mxGetScalar(prhs[1]);
    r = mxGetScalar(prhs[2]);

    if (nrhs >= 4) {
        returnToInitial = (bool)mxGetScalar(prhs[3]);
    }
    if (nrhs >= 5) {
        char buf[100];
        if (mxGetString(prhs[4], buf, 100) != 0) {
            mexErrMsgIdAndTxt(MEX_MODULE_NAME ":algorithm:parsingName",
                              "Failed reading algorithm name.");
        }
        algorithmName = buf;
    }

    // Construct the graph
    Graph G;
    GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES); 
    unpackNodes(G, GA, V, n);
    node u;

    #ifdef MEX_DEBUG
    mexPrintf("Printing graph...\n");
    forall_nodes(u, G) {
        mexPrintf("Node %d at (%0.1f, %0.1f)\n",GA.idNode(u),GA.x(u),GA.y(u));
    }
    mexPrintf("\n");

    dpp::Logger::logDebug(DPP_LOGGER_VERBOSE_3) << dpp::printGraph(G, GA);
    #endif

    // Set algorithm
    dpp::DtspPlanningAlgorithm alg;
    if (algorithmName.compare("alternating") == 0) {
        alg = dpp::DtspPlanningAlgorithm::ALTERNATING;
    }
    else if (algorithmName.compare("nearest") == 0) {
        alg = dpp::DtspPlanningAlgorithm::NEAREST_NEIGHBOR;
    }
    else if (algorithmName.compare("randomized") == 0) {
        alg = dpp::DtspPlanningAlgorithm::RANDOMIZED;
    }
    else {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":algorithm:unknownName",
                          "Unknown algorithm name given.");
    }

    // Call solver
    List<node> Tour;
    List<edge> Edges;
    NodeArray<double> Headings(G,0.0);
    int result = solveDtsp(G, GA, x, r, Tour, Edges, Headings, cost, returnToInitial,
        alg);

    if (result != SUCCESS) {
        char buf[50];
        sprintf(buf, "Solver failed with code: %d", result);
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":solver:failure", buf);
    }
    int m = Edges.size();

    // Build output matrices
    plhs[0] = mxCreateDoubleMatrix((mwSize)m,(mwSize)3,mxREAL);
    plhs[1] = mxCreateDoubleMatrix((mwSize)n,(mwSize)1,mxREAL);
    plhs[2] = mxCreateDoubleScalar(cost);
    double *pE = mxGetPr(plhs[0]);
    double *pX = mxGetPr(plhs[1]);

    // Pack outputs
    #ifdef MEX_DEBUG
    mexPrintf("Creating output tour edges m=%d, and headings n=%d\n",m,n);
    mexEvalString("drawnow");
    #endif

    packEdges(G, GA, Edges, pE);

    packHeadings(G, GA, Headings, pX);
}