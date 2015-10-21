#ifndef _DPP_ALGORITHM_H_
#define _DPP_ALGORITHM_H_

#include <string>
#include <stdbool.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

using ogdf::node;
using ogdf::edge;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::NodeArray;

namespace dpp {

/*
 * Base class for solving.
 */
class Algorithm {
public:
    enum Type {DTSP, CPP};

    Algorithm(std::string name, Type t)
        : m_name(name),
        m_type(t)
    { }

    virtual ~Algorithm() = 0;

    static int runLKHSolver(std::string parFilename);

    const Type type(void) {
        return m_type;
    }

    const std::string typeText(void) {
        return TypeText[m_type];
    }

    const std::string name(void) {
        return m_name;
    }

protected:
    static const char *TypeText[];
    const std::string m_name;
    const Type m_type;
};


/*
 * Dubins Traveling Salesperson Problem algorithm.
 */
class AlgorithmDtsp : public Algorithm {
public:
    AlgorithmDtsp(std::string name)
        : Algorithm(name, Algorithm::Type::DTSP)
    { }

    ~AlgorithmDtsp() 
    { }

    virtual int run(Graph &G, GraphAttributes &GA, double x, double r,
        List<node> &Tour, List<edge> &Edges, NodeArray<double> &Headings, double &cost,
        bool returnToInitial=true) = 0;
};

/*
 * Coverage Path Planning algorithm.
 */
class AlgorithmCpp : public Algorithm {
public:
    AlgorithmCpp(std::string name)
        : Algorithm(name, Algorithm::Type::CPP)
    { }

    ~AlgorithmCpp() {
    }

    virtual int run(void) = 0;
};


} // namespace dpp

#endif // _DPP_ALGORITHM_H_