/*
The MIT License
Copyright (c) 2015 UCSC Autonomous Systems Lab
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <stdio.h>
#include "mex.h"

#define USE_MEX_MODE 
#include "alternatingDTSP.cpp"
#include "TSPLib.h"

using namespace std;


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{

    /* check for proper number of arguments */
    if(nrhs!=3) {
    mexErrMsgIdAndTxt("DubinsAreaCoverage:path:nrhs",
                      "Three inputs required.");
    }

    if(nlhs!=3) {
    mexErrMsgIdAndTxt("DubinsAreaCoverage:path:nlhs",
                      "Three outputs required.");
    }

    // check that columns in V is 2 
    if(mxGetN(prhs[0])!=2) {
        mexErrMsgIdAndTxt("DubinsAreaCoverage:V:wrongColumns",
                          "V must have 2 columns.");
    }

    // check that rows in V is larger than 1
    if(mxGetM(prhs[0])<2) {
        mexErrMsgIdAndTxt("DubinsAreaCoverage:V:notEnoughRows",
                          "V must have at least 2 rows.");
    }

    // Ensure x and r are scalar Doubles
    if( !mxIsDouble(prhs[1]) || 
         mxIsComplex(prhs[1]) ||
         mxGetNumberOfElements(prhs[1])!=1 ) {
        mexErrMsgIdAndTxt("DubinsAreaCoverage:x:notScalar",
                          "Input heading must be a scalar.");
    }

    if( !mxIsDouble(prhs[2]) || 
         mxIsComplex(prhs[2]) ||
         mxGetNumberOfElements(prhs[2])!=1 ) {
        mexErrMsgIdAndTxt("DubinsAreaCoverage:r:notScalar",
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
    NodeArray<double> heading(G,0.0);
    int result = solveAlternatingDTSP(G, GA, x, r, tour, heading, cost);

    if (result != 0) {
        mexErrMsgIdAndTxt("DubinsAreaCoverage:solver:failure",
            "Solver failed with a non-zero return code.");
    }

    int m = tour.size() - 1;

    // Build output matrices
    plhs[0] = mxCreateDoubleMatrix((mwSize)m,(mwSize)2,mxREAL);
    plhs[1] = mxCreateDoubleMatrix((mwSize)1,(mwSize)n,mxREAL);
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
    for ( ; (row < m && iter != tour.end()) ; iter++) {
        node u = *iter;
        node v = *(iter.succ());
        int u_id = GA.idNode(u), v_id = GA.idNode(v);

        #ifdef DEBUG
        mexPrintf("Node %d to %d.\n", u_id, v_id);
        #endif

        Mat(pE,row,0) = u_id;
        Mat(pE,row,1) = v_id;
        row++;
    }

    // Copy headings
    int i = 0;
    forall_nodes(u, G) {
        int u_idx = GA.idNode(u) - 1;
        pX[u_idx] = heading[u];
    }
}
