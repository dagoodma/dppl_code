/*
 * MEX interface for the solveCppAsDtsp binary.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <string>

#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/basic/VehicleConfiguration.h>

#include "mexHelper.h"
#include "solveCppAsDtsp.h"

#define LOG_FILENAME "solveCppAsDtsp.log"

/*
 * MEX gateway function. The MEX file synopsis is as follows:
 *
 *      solveCppAsDtsp P C r e [return] [algorithm]
 *
 * Parameters:
 *      P   Points of polygon, n-by-2 matrix of vertex positions
 *      C   Initial configuration, 3-by-1 vector of x, y, and heading [rad]
 *      r   Vehicle turn radius [m]
 *      e   Sensor coverage width [m]
 *      [return]      Optional argument whether to return to the starting node
 *      [algorithm]   Optional DTSP algorithm name. Options are "alternating",
 *                    "nearest", and "randomized".
 * Returns:
 *      V       Vertices of coverage graph, n-by-2 matrix with rows: x, y
 *      E       Edges of tour, m-by-3 matrix with rows: srcNodeId, targNodeId, cost
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
    mexPrintf("Logging to " LOG_FILENAME ".\n");
    #endif

    /* check for proper number of arguments */
    if(nrhs<4) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nrhs",
                      "At least four inputs are required.");
    }
    if(nrhs>6) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nrhs",
                      "Too many input arguments given.");
    }

    if(nlhs!=4) {
    mexErrMsgIdAndTxt(MEX_MODULE_NAME ":path:nlhs",
                      "Four outputs required.");
    }

    // check that columns in P is 2 
    if(mxGetN(prhs[0])!=2) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":P:wrongColumns",
                          "P must have 2 columns.");
    }

    // check that rows in P is larger than 3
    if(mxGetM(prhs[0])<3) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":P:notEnoughRows",
                          "P must have at least 3 rows.");
    }

    // check that C is 3-by-1 or 1-by-3
    //if(!((mxGetN(prhs[1]) == 3 && mxGetM(prhs[1]) == 1)
    //    || (mxGetN(prhs[1]) == 1 && mxGetM(prhs[1]) == 3))) {
    if (mxGetNumberOfElements(prhs[1])!=3 ) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":C:wrongDimensions",
                          "C must be 3-by-1 or 1-by-3.");
    }

    // TODO check that heading in C is between [0, 2*pi)

    // Ensure r and e are scalar Doubles
    if( !mxIsDouble(prhs[2]) || 
         mxIsComplex(prhs[2]) ||
         mxGetNumberOfElements(prhs[2])!=1 ) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":r:notScalar",
                          "Input turn radius must be a scalar.");
    }

    if( !mxIsDouble(prhs[3]) || 
         mxIsComplex(prhs[3]) ||
         mxGetNumberOfElements(prhs[3])!=1 ) {
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":e:notScalar",
                          "Input sensor width must be a scalar.");
    }

    size_t nP = mxGetM(prhs[0]);

    // Input variables
    double *P, *C, r, e;             // input variables
    bool returnToInitial = false;
    std::string algorithmName("alternating");

    P = mxGetPr(prhs[0]);
    C = mxGetPr(prhs[1]);
    r = mxGetScalar(prhs[2]);
    e = mxGetScalar(prhs[3]);

    // Handle the extra arguments (returnToInitial)
    if (nrhs >= 5) {
        returnToInitial = (bool)mxGetScalar(prhs[4]);
    }
    // ... extra args (algorithm)
    if (nrhs >= 6) {
        char buf[100];
        if (mxGetString(prhs[5], buf, 100) != 0) {
            mexErrMsgIdAndTxt(MEX_MODULE_NAME ":algorithm:parsingName",
                              "Failed reading algorithm name.");
        }
        algorithmName = buf;
    }

    // Create polygon object from vertices
    DPolygon polygon;
    unpackPolygon(polygon, P, nP);
    #ifdef MEX_DEBUG
    mexPrintf("Created polygon with %d points.\n", polygon.size());
    #endif

    // Create initial vehicle config
    dpp::VehicleConfiguration VC(C[0], C[1], C[2]);
    #ifdef MEX_DEBUG
    dpp::Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Initial position ("
        << VC.x() << ", " << VC.y() << ") with heading: " << VC.heading() << std::endl; 
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
    Graph G;
    GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES); 
    List<node> Tour;
    List<edge> Edges;
    NodeArray<double> Headings(G,0.0);
    double cost;
    int result = solveCppAsDtsp(polygon, VC, G, GA, r, e, Tour, Edges, Headings, cost, returnToInitial,
        alg);

    if (result != SUCCESS) {
        char buf[50];
        sprintf(buf, "Solver failed with code: %d", result);
        mexErrMsgIdAndTxt(MEX_MODULE_NAME ":solver:failure", buf);
    }
    int n = G.numberOfNodes();
    int m = Edges.size();

    // Output variables
    double *pV, *pE, *pX;  
    plhs[0] = mxCreateDoubleMatrix((mwSize)n,(mwSize)2,mxREAL);
    plhs[1] = mxCreateDoubleMatrix((mwSize)m,(mwSize)3,mxREAL);
    plhs[2] = mxCreateDoubleMatrix((mwSize)n,(mwSize)1,mxREAL);
    plhs[3] = mxCreateDoubleScalar(cost);
    pV = mxGetPr(plhs[0]);
    pE = mxGetPr(plhs[1]);
    pX = mxGetPr(plhs[2]);

    // Pack outputs
    #ifdef MEX_DEBUG
    mexPrintf("Creating output tour edges m=%d, and headings n=%d\n",m,n);
    mexEvalString("drawnow");
    #endif

    packNodes(G, GA, pV);

    packEdges(G, GA, Edges, pE);

    packHeadings(G, GA, Headings, pX);

}