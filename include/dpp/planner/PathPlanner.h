#ifndef _DPP_PATH_PLANNER_H_
#define _DPP_PATH_PLANNER_H_

#include <memory>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/planalg/Algorithm.h>

namespace dpp {
/*
 * Base class for a path planner.
 */
class PathPlanner {
public:
    PathPlanner(Algorithm *alg)
      : m_G(),
        m_GA(m_G,
          GraphAttributes::nodeGraphics |
          GraphAttributes::edgeGraphics |
          GraphAttributes::nodeLabel |
          GraphAttributes::edgeStyle |
          GraphAttributes::edgeDoubleWeight |
          GraphAttributes::nodeStyle |
          GraphAttributes::nodeTemplate |
          GraphAttributes::nodeId),
        m_haveSolution(false),
        m_returnToInitial(true),
        m_initialHeading(0.0),
        m_cost(-1),
        m_Tour(),
        m_Edges(),
        m_Headings(m_G),
        m_algorithm(alg) {
    }

    virtual ~PathPlanner()  = 0;

    Graph graph(void) {
        return m_G;
    }

    GraphAttributes graphAttributes(void) {
        return m_GA;
    }

    List<node> tour(void) {
        DPP_ASSERT(m_haveSolution);
        return m_Tour;
    }

    List<edge> edges(void) {
        DPP_ASSERT(m_haveSolution);
        return m_Edges;
    }

    NodeArray<double> headings(void) {
        return m_Headings;
    }

    void headings(NodeArray<double> X) {
        DPP_ASSERT(X.graphOf() == const_cast<const Graph*>(&m_G));
        m_Headings = X;
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

    void initialHeading(double x) {
        m_initialHeading = x;
    }

    double initialHeading(void) {
        return m_initialHeading;
    }

    virtual bool solve(void) = 0;

protected:
    std::unique_ptr<dpp::Algorithm> m_algorithm;

    Graph m_G;
    GraphAttributes m_GA;
    List<node> m_Tour;
    List<edge> m_Edges;
    NodeArray<double> m_Headings;

    double m_cost;
    bool m_haveSolution;
    bool m_returnToInitial;
    double m_initialHeading;

};

} // namespace dpp

#endif // _DPP_PATH_PLANNER_H_