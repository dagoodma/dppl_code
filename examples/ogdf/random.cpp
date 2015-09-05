#include <ogdf/basic/Graph.h>
#include <ogdf/basic/graph_generators.h>
#include <ogdf/layered/DfsAcyclicSubgraph.h>
#include <ogdf/fileformats/GraphIO.h>
 
using namespace ogdf;
 
int main()
{
    Graph G;
    randomSimpleGraph(G, 10, 20);
    DfsAcyclicSubgraph DAS;
    DAS.callAndReverse(G);
    //GraphIO::drawSVG(G, "test.svg");
    //GraphIO::writeGML(G, "test.gml");
    GraphIO::writeDOT(G, "random_graph.dot");
 
    return 0;
}


