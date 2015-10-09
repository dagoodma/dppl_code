#ifndef _DPP_NODE_MATRIX_H_
#define _DPP_NODE_MATRIX_H_

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <dpp/basic/basic.h>

namespace dpp {

template <class T>
class NodeMatrix
{
public:
    NodeMatrix() {
        m_nodes = 0;
    }
 
    NodeMatrix( ogdf::Graph &G ) {
        allocate( G );
    }

    ~NodeMatrix() {
        deallocate();
    }

    ogdf::NodeArray<T> & operator[](const ogdf::node &v) {
        return (*m_data)[v];
        //return ogdf::NodeArray<T>::operator [](v->index());
    }

    T get( const ogdf::node &i, const ogdf::node &j ) {
        return (*m_data)[i][j];
    }

    void set( const T& t, const ogdf::node &i, const ogdf::node &j) {
        (*m_data)[i][j] = t;
    }

    void setAll( const T& t) {
        if (m_graph == nullptr)
            return;

        ogdf::node i, j;
        forall_nodes(i, *m_graph) {
            forall_nodes(j, *m_graph) {
                (*m_data)[i][j] = t;
            }
        }
    }

    int numberOfNodes() {
        return m_nodes;
    }

    const ogdf::Graph * graphOf() {
        return m_graph;
    }

private:
    void allocate( ogdf::Graph &G ) {
        m_nodes = G.numberOfNodes();
        m_graph = &G;

        m_data = new ogdf::NodeArray<ogdf::NodeArray<T>>(G);

        ogdf::node i;
        forall_nodes(i, G) {
            (*m_data)[i] = ogdf::NodeArray<T>(G);
        }

        setAll(0.0);
    }

    void deallocate() {
        delete m_data;
        m_nodes = 0;
    }

    // Attributes
    int m_nodes;
    ogdf::NodeArray<ogdf::NodeArray<double>> *m_data;
    const ogdf::Graph *m_graph;
};

} // namespace dpp

#endif // _DPP_NODE_MATRIX_H_

