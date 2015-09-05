#include <Snap.h>

int main()
{
    // create a graph
    PNGraph Graph = TNGraph::New();
    Graph->AddNode(1);
    Graph->AddNode(5);
    Graph->AddNode(32);
    Graph->AddEdge(1,5);
    Graph->AddEdge(5,1);
    Graph->AddEdge(5,32);

    // print graph
    TFOut FOut = TFOut::New("directed_graph1.graph");
    Graph->Save(FOut)

    return 0;
}
