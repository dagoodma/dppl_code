/*
 * Copyright (C) 2014-2015 DubinsSensorCoverage.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the COPYRIGHT file distributed with DubinsSensorCoverage.
*/
#include "solveDTSP.h"

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{

    /* check for proper number of arguments */
    if(nrhs!=3) {
    mexErrMsgIdAndTxt("DubinsSensorCoverage:path:nrhs",
                      "Three inputs required.");
    }

    if(nlhs!=3) {
    mexErrMsgIdAndTxt("DubinsSensorCoverage:path:nlhs",
                      "Three outputs required.");
    }

    // check that columns in V is 2 
    if(mxGetN(prhs[0])!=2) {
        mexErrMsgIdAndTxt("DubinsSensorCoverage:V:wrongColumns",
                          "V must have 2 columns.");
    }

    // check that rows in V is larger than 1
    if(mxGetM(prhs[0])<2) {
        mexErrMsgIdAndTxt("DubinsSensorCoverage:V:notEnoughRows",
                          "V must have at least 2 rows.");
    }

    // Ensure x and r are scalar Doubles
    if( !mxIsDouble(prhs[1]) || 
         mxIsComplex(prhs[1]) ||
         mxGetNumberOfElements(prhs[1])!=1 ) {
        mexErrMsgIdAndTxt("DubinsSensorCoverage:x:notScalar",
                          "Input heading must be a scalar.");
    }

    if( !mxIsDouble(prhs[2]) || 
         mxIsComplex(prhs[2]) ||
         mxGetNumberOfElements(prhs[2])!=1 ) {
        mexErrMsgIdAndTxt("DubinsSensorCoverage:r:notScalar",
                          "Input radius must be a scalar.");
    }

    // Build graph from V
    size_t n = mxGetM(prhs[0]);
    size_t cols = mxGetN(prhs[0]);
    double *v, x, r;             // input variables
    double *E, *X, cost;         // output variables

    v = mxGetPr(prhs[0]);
    x = mxGetScalar(prhs[1]);
    r = mxGetScalar(prhs[2]);

    Graph G;
    GraphAttributes GA(G,
      GraphAttributes::nodeGraphics |
      GraphAttributes::edgeGraphics |
      GraphAttributes::nodeLabel |
      GraphAttributes::edgeStyle |
      GraphAttributes::edgeDoubleWeight |
      GraphAttributes::nodeStyle |
      GraphAttributes::nodeTemplate |
      GraphAttributes::nodeId); 

    #ifdef DEBUG
    mexPrintf("Building a graph with %d nodes.\n", n);
    mexEvalString("drawnow");
    #endif
    
  #define Mat(a,i,j) a[(i)+(j)*n] 
    int nodesCreated = 0;
    for (int i=0; i < n; i++) {
        node u = G.newNode();
        GA.x(u) = Mat(v,i,0);
        GA.y(u) = Mat(v,i,1);
        GA.idNode(u) = nodesCreated+1;
        nodesCreated++;
    }

    // Call solver
    List<node> tour;
    List<edge> edges;
    NodeArray<double> heading(G,0.0);
    int result = solveRandomizedDTSP(G, GA, x, r, tour, edges, heading, cost);

    if (result != 0) {
        char buf[50];
        sprintf(buf, "Solver failed with code: %d", result);
        mexErrMsgIdAndTxt("DubinsSensorCoverage:solver:failure", buf);
    }

    int m = edges.size();

    // Build output matrices
    plhs[0] = mxCreateDoubleMatrix((mwSize)m,(mwSize)3,mxREAL);
    plhs[1] = mxCreateDoubleMatrix((mwSize)n,(mwSize)1,mxREAL);
    plhs[2] = mxCreateDoubleScalar(cost);
    double *pE = mxGetPr(plhs[0]), *pX = mxGetPr(plhs[1]);
    node u;
   
    #ifdef DEBUG
    mexPrintf("Creating output tour edges m=%d, and headings n=%d\n",m,n);
    mexEvalString("drawnow");
    #endif

    #ifdef DEBUG
    mexPrintf("Printing graph...\n");
    forall_nodes(u, G) {
        mexPrintf("Node %d at (%0.1f, %0.1f)\n",GA.idNode(u),GA.x(u),GA.y(u));
    }
    mexPrintf("\n");
    #endif

    // Iterate over the tour and build the edge output matrix E
    ListIterator<node> iter = tour.begin();
    #ifdef DEBUG
    mexPrintf("Printing tour...\n");
    #endif
    int row = 0;
    edge e;
    forall_edges(e, G) {
        node u = e->source();
        node v = e->target();
        int u_id = GA.idNode(u);
        int v_id = GA.idNode(v);

        #ifdef DEBUG
        mexPrintf("Edge is 0X%X with weight %0.1f.\n", e, GA.doubleWeight(e));
        mexPrintf("Node %d to %d.\n", u_id, v_id);
        #endif

        Mat(pE,row,0) = u_id;
        Mat(pE,row,1) = v_id;
        Mat(pE,row,2) = GA.doubleWeight(e);
        row++;
    }

    // Copy headings
    int i = 0;
    forall_nodes(u, G) {
        int u_idx = GA.idNode(u) - 1;
        pX[u_idx] = heading[u];
    }
}
