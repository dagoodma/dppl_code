#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>

using namespace ogdf;

int main()
{
    Graph G;
    GraphAttributes GA(G, GraphAttributes::nodeGraphics | GraphAttributes::edgeGraphics );

    const int LEN = 11;
    for(int i = 1; i<LEN; ++i) {
        node left = G.newNode();
        node left2 = G.newNode();
        cout << "Left degree = " << left->degree() << ".\n";
        cout << "Left uid = " << left->index() << ".\n";
        cout << "Left2 uid = " << left2->index() << ".\n";
        /*
         Code fails here:
            Assertion failed: (v->graphOf() == m_pGraph), function operator[],
                file /usr/local/include/ogdf/basic/NodeArray.h, line 184.
            Abort trap: 6
         */

        cout << "Here3\n";
        //int xl = GA.x(left);
        cout << "Here1\n";
        //xl = -5*(i+1);
        cout << "Here2\n";
        GA.y(left) = -20*i;
        GA.width(left) = 10*(i+1);
        GA.height(left) = 15;

        node bottom = G.newNode();
        cout << "Here3\n";
        GA.x(bottom) = 20*(LEN-i);
        cout << "Here4\n";
        GA.y(bottom) = 5*(LEN+1-i);
        GA.width(bottom) = 15;
        GA.height(bottom) = 10*(LEN+1-i);

        edge e = G.newEdge(left,bottom);
        DPolyline &p = GA.bends(e);
        p.pushBack(DPoint(10,-20*i));
        p.pushBack(DPoint(20*(LEN-i),-10));
    }

    //GraphIO::drawSVG(GA, "manual_graph.svg");
    //GraphIO::writeGML(GA, "manual_graph.gml");
    GraphIO::writeDOT(GA, "manual_graph.dot");

    return 0;
}
