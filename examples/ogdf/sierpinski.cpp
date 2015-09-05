/**
*This is the solution file for the Exercise 02
*
* @author Chameera Wijebandara
* @email chameerawijebandara@gmail.com
*
*   Windows 7
*   Vishual stuio 2010
*   OGDF Snapshot 2014-02-28
*
*/
 
#include <ogdf/basic/Graph.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/basic/graph_generators.h>
#include <ogdf/layered/SugiyamaLayout.h>
#include <ogdf/layered/OptimalRanking.h>
#include <ogdf/layered/FastHierarchyLayout.h>
#include <ogdf/layered/BarycenterHeuristic.h>
 
# define NODESIZE 10
using namespace ogdf;
 
bool sierpinksiGraph ( const int n );
void initSierpinksiGraph( Graph *G, GraphAttributes *GA, float points [3][2] );
void genarateSierpinksiGraph( Graph *G, GraphAttributes *GA, float parent[3][2], int count );
 
// main
int main()
{
    cout<<"Pleace enter number n (n>0) : ";
    int n;
    cin >> n;
    sierpinksiGraph(n);
    return 0;
}
//end of the main
 
/*
A function sierpinksiGraph that creates a Sierpinksi graph of order n, where n is a function parameter.
and writes the final drawing to a SVG file
graph generator for Sierpinski graphs
*/
 
bool sierpinksiGraph ( const int n )//number of nodes n and the number of edges m of the graph to be generated
{
    if( n < 1 ) // if input is invalid
    {
        return false;
    }
 
    Graph G; // inti garaph
    GraphAttributes GA(G, GraphAttributes::nodeGraphics |
        GraphAttributes::nodeStyle |
        GraphAttributes::nodeLabel |
        GraphAttributes::edgeGraphics |
        GraphAttributes::edgeArrow |
        GraphAttributes::edgeStyle );
 
    float points [ 3 ][ 2 ] = {
        { 0, 0 },
        { 50*(n+3), 87*(n+3) },
        { -50*(n+3), 87*(n+3) }}; //  points  of the outer triangerl
 
        initSierpinksiGraph( &G, &GA, points );  // init the  Sierpinski graphs
        genarateSierpinksiGraph( &G, &GA, points, n-1 ); // recursively call pirates of the graph
 
        GraphIO::drawSVG( GA, "<code>mst.svg</code>" ); // write in to file
 
        return true;
}
 
/*
initialize the outer triangle
*/
void initSierpinksiGraph(Graph *G, GraphAttributes *GA,float points [3][2])
{
 
    for(int i=0;i<3;i++) // 3 points in thetriangle
    {
        node v  = G->newNode();
        cout << "Here1" << GA->x(v) << "\n";
        //GA->x(v) = points[i][0];
        cout << "Here2\n";
        GA->y(v) = points[i][1];
        GA->width(v) = NODESIZE;
        GA->height(v) = NODESIZE;
        GA->shape(v) = ogdf::Shape::shEllipse;
        GA->fillColor(v) = Color("#FF0000"); // set colour
        GA->strokeColor(v) = Color("#FF0000");
 
        node u;
        forall_nodes(u, *G){
            edge e = G->newEdge(v,u);
            GA->bends(e);
            GA->arrowType(e) =  ogdf::EdgeArrow::eaNone;
            GA->strokeColor(e) = Color("#FF0000");
        }
    }
 
}
 
/*
recursively grow the  graph
*/
void genarateSierpinksiGraph(Graph *G, GraphAttributes *GA,float parent[3][2],int count)
{
    if(count==0)
    {
        return;
    }
 
    node list[3];
    for(int i=0;i<3;i++)
    {
        // create new node
        node v  = G->newNode();
        GA->x(v) = (parent[i][0] + parent[(i+1)%3][0])/2;
        GA->y(v) = (parent[i][1] + parent[(i+1)%3][1])/2;
        GA->width(v) = NODESIZE;
        GA->height(v) = NODESIZE;
        GA->shape(v) = ogdf::Shape::shEllipse;
        GA->fillColor(v) = Color("#FF0000");
 
        list[i] = v;
 
        // create new edge
        for(int j=0;j<i;j++){
 
            edge e = G->newEdge(v,list[j]);
            GA->bends(e);
            GA->arrowType(e) =  ogdf::EdgeArrow::eaNone;
            GA->strokeColor(e) = Color("#FF0000");
        }
 
        float child[3][2] = {
            {parent[i][0],parent[i][1]},
            {(parent[i][0] + parent[(i+1)%3][0])/2,(parent[i][1] + parent[(i+1)%3][1])/2},
            {(parent[i][0] + parent[(i+2)%3][0])/2,(parent[i][1] + parent[(i+2)%3][1])/2}};
 
            // call for next iteration
            genarateSierpinksiGraph(G,GA,child,count-1);
    }
}
