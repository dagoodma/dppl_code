#ifndef _DPP_PATH_PLANNER_H_
#define _DPP_PATH_PLANNER_H_

#include <memory>

#include <ogdf/basic/EdgeArray.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Util.h>
#include <dpp/basic/Logger.h>
#include <dpp/planalg/Algorithm.h>

using ogdf::EdgeArray;

namespace dpp {
/*
 * Base class for a path planner.
 */
class PathPlanner {
public:
    PathPlanner(Algorithm *alg)
      : m_G(),
        m_GA(m_G, DPP_GRAPH_ATTRIBUTES),
        m_haveSolution(false),
        m_returnToInitial(true),
        m_cost(0),
        m_Tour(),
        m_Edges(),
        m_algorithm(alg) {
    }

    virtual ~PathPlanner() = 0;

    const std::string algorithmName(void) {
        return m_algorithm->name();
    }

    Graph *graphPtr(void) {
        return &m_G;
    }

    GraphAttributes &graphAttributes(void) {
        return m_GA;
    }

    List<node> &tour(void) {
        DPP_ASSERT(m_haveSolution);
        return m_Tour;
    }

    List<edge> &edges(void) {
        DPP_ASSERT(m_haveSolution);
        return m_Edges;
    }

    double cost(void) {
        DPP_ASSERT(m_haveSolution);
        return m_cost;
    }

    int waypointCount(void) {
        return m_G.numberOfNodes();
    }

    void returnToInitial(bool value) {
        m_returnToInitial = value;
    }

    bool returnToInitial(void) {
        return m_returnToInitial;
    }

    virtual bool solve(void) = 0;

    bool haveSolution(void) {
        return m_haveSolution;
    }

protected:
    std::unique_ptr<dpp::Algorithm> m_algorithm;

    Graph m_G;
    GraphAttributes m_GA;
    List<node> m_Tour;
    List<edge> m_Edges;

    double m_cost;
    bool m_haveSolution;
    bool m_returnToInitial;

};

/**
 * Base class for a Dubins vehicle path planner (supports constrained turning radius).
 */
class DubinsPathPlanner : public PathPlanner {
public:

    DubinsPathPlanner(double turnRadius, Algorithm *alg)
        : m_turnRadius(turnRadius),
        m_initialHeading(0.0),
        m_Headings(m_G),
        PathPlanner(alg)
    { }

    virtual ~DubinsPathPlanner() = 0;

    void initialHeading(double x) {
        m_initialHeading = x;
    }

    double initialHeading(void) {
        return m_initialHeading;
    }

    double turnRadius(void) {
        return m_turnRadius;
    }

    void turnRadius(double r) {
        m_turnRadius = r;
    }

    NodeArray<double> &headings(void) {
        DPP_ASSERT(m_Headings.graphOf() == const_cast<const Graph*>(&m_G));
        return m_Headings;
    }

    void headings(NodeArray<double> &X) {
        DPP_ASSERT(X.graphOf() == const_cast<const Graph*>(&m_G));
        m_Headings = X;
    }

    void copySolution(ogdf::Graph &G, ogdf::GraphAttributes &GA,
        ogdf::List<ogdf::node> &Tour, ogdf::List<ogdf::edge> &Edges,
        NodeArray<double> &Headings, double &cost);

    virtual bool solve(void) = 0;
protected:
    double m_turnRadius;
    double m_initialHeading;
    NodeArray<double> m_Headings;

}; // DubinsPathPlanner

} // namespace dpp

#endif // _DPP_PATH_PLANNER_H_