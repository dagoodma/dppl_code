/*
 * MATLAB MEX helper functions.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
 #include "mexHelper.h"


/**
 * Pack the graphs nodes into the matrix of vertices V.
 * @param G     The graph to pack into V.
 * @param GA    Attributes of the graph.
 * @param pV    A pointer to an n-by-2 matrix of vertices.
 * @return      Number of nodes packed into V.
 * @note pV should be allocated as n-by-2, where n is the number of nodes in G.
 */
int packNodes(Graph &G, GraphAttributes &GA, double *pV) {
    node u;
    int i = 0;
    int n = G.numberOfNodes();
    forall_nodes(u,G) {
        MEX_MAT(pV, i, 0, n) = GA.x(u);
        MEX_MAT(pV, i, 1, n) = GA.y(u);
        i++;
    }

    return i;
}

/**
 * Adds vertices to the polygon by unpacking P.
 * @param polygon   Polygon to add points to.
 * @param pP        A pointer to an n-by-2 matrix of polygon vertices.
 * @param n         Number of rows in pP.
 * @return          Number of points added to the polygon.
 */
int unpackPolygon(DPolygon &polygon, double *pP, int n) {

    #ifdef MEX_DEBUG
    mexPrintf("Printing polygon vertices...\n");
    #endif
    int i;
    for (i = 0; i < n; i++) {
        double x = MEX_MAT(pP, i, 0, n);
        double y = MEX_MAT(pP, i, 1, n);

        #ifdef MEX_DEBUG
        mexPrintf("Vertex %d at (%0.1f, %0.1f)\n", i + 1, x, y);
        #endif

        DPoint p(x,y);
        polygon.pushBack(p);
    }

    return i;
}

/**
 * Add nodes to the graph by unpacking V.
 * @param G     Graph.
 * @param GA    Attributes for the graph.
 * @param pV    A pointer to an n-by-2 matrix of vertices as input.
 * @param n     Number of rows in V.
 * @return      Number of nodes added to the graph.
 */
int unpackNodes(Graph &G, GraphAttributes &GA, double *pV, int n) {

    #ifdef MEX_DEBUG
    mexPrintf("Building a graph with %d nodes.\n", n);
    mexEvalString("drawnow");
    #endif
    
    int nodesCreated = 0;
    for (int i=0; i < n; i++) {
        node u = G.newNode();
        GA.x(u) = MEX_MAT(pV,i,0,n);
        GA.y(u) = MEX_MAT(pV,i,1,n);
        GA.idNode(u) = nodesCreated+1;
        nodesCreated++;
    }

    return nodesCreated;
}

/**
 * Pack Edges into a m-by-3 matrix.
 * @param G     Graph.
 * @param GA    Attributes for the graph.
 * @param Edges An m-by-3 matrix of edges.
 * @param pE    Pointer to m-by-3- matrix of edges for output.
 * @return      Number of edges added.
 */
int packEdges(Graph &G, GraphAttributes &GA, List<edge> &Edges, double *pE) {
    int m = Edges.size();
    #ifdef MEX_DEBUG
    mexPrintf("Printing tour...\n");
    #endif
    int row = 0;
    edge e;
    forall_edges(e, G) {
        node u = e->source();
        node v = e->target();
        int u_id = GA.idNode(u);
        int v_id = GA.idNode(v);

        #ifdef MEX_DEBUG
        mexPrintf("Edge is 0X%X with weight %0.1f.\n", e, GA.doubleWeight(e));
        mexPrintf("Node %d to %d.\n", u_id, v_id);
        #endif

        MEX_MAT(pE,row,0,m) = u_id;
        MEX_MAT(pE,row,1,m) = v_id;
        MEX_MAT(pE,row,2,m) = GA.doubleWeight(e);
        row++;
    }

    return row;
}

/**
 * Pack Headings into an n-by-1 vector.
 * @param G     Graph.
 * @param GA    Attributes for the graph.
 * @param X     An n-by-1 vector of node headings.
 * @param pX    Pointer to n-by-1 vector of headings for output.
 * @return      Number of headings added.
 */
int packHeadings(Graph &G, GraphAttributes &GA, NodeArray<double> &X, double *pX) {
    // Copy headings
    int i = 0;
    node u;
    forall_nodes(u, G) {
        int u_idx = GA.idNode(u) - 1;
        pX[u_idx] = X[u];
        i++;
    }
    return i;
}
