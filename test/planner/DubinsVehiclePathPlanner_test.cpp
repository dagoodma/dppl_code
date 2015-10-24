/*
 * Unit test for dpp/planner/DubinsVehiclePathPlanner.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <iostream>
#include <vector>
#include <climits>

#include <gtest/gtest.h>

#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

using ogdf::DPoint;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::ListConstIterator;

#define TURN_RADIUS     1.0
#define INPUT_FILEPATH  "../data/triangle-8wp.gml"
#define INITIAL_HEADING 0.0

typedef dpp::DubinsVehiclePathPlanner::DtspPlanningAlgorithm DtspPlanningAlgorithm;

 // Test dies without solution
TEST(DubinsVehiclePathPlannerDeathTest, DeathWithoutSolution) {
    dpp::DubinsVehiclePathPlanner planner;
    ASSERT_EQ(0, planner.waypointCount());
    ASSERT_EQ(0, planner.haveSolution());

    EXPECT_DEATH(planner.tour(), "Assertion failed: \\(m_haveSolution\\).*");
    EXPECT_DEATH(planner.edges(), "Assertion failed: \\(m_haveSolution\\).*");
    EXPECT_DEATH(planner.cost(), "Assertion failed: \\(m_haveSolution\\).*");
}

// Test enforces headings/attributes are of graph
TEST(DubinsVehiclePathPlannerTest, MembersAreOfGraph) {
    dpp::DubinsVehiclePathPlanner planner;
    NodeArray<double> Headings = planner.headings();
    Graph *pG = planner.graphPtr();
    GraphAttributes GA = planner.graphAttributes();

    // Headings and attributes are of graph
    EXPECT_EQ(pG, Headings.graphOf());
    EXPECT_EQ(pG, &(GA.constGraph()));

    // Can't set headings not of graph
    Graph Gnew;
    NodeArray<double> HeadingsNew(Gnew);
    EXPECT_DEATH(planner.headings(HeadingsNew), "Assertion failed: \\(X\\.graphOf\\(\\) == const_cast<const Graph\\*>\\(&m_G\\)\\).*");
}

using ::testing::Test;
using ::testing::Values;
using ::testing::ValuesIn;

class DubinsVehiclePathPlannerFeatureTest : public Test {
public:
    DubinsVehiclePathPlannerFeatureTest()
        : m_filename(INPUT_FILEPATH)
    { }

    virtual ~DubinsVehiclePathPlannerFeatureTest() { }

    virtual void SetUp() {
        m_planner.initialHeading(INITIAL_HEADING);
        m_planner.turnRadius(TURN_RADIUS);
    } // SetUp()

    virtual void TearDown() {
    } // TearDown()

    bool solve(void) {
        return m_planner.solve();
    }

    void initializeWaypoints(const List<DPoint> &nodes) {
        int n = nodes.size();

        // Build graph
        Graph G;
        GraphAttributes GA(G,DPP_GRAPH_ATTRIBUTES);

        // Populate graph
        ListConstIterator<DPoint> iter;
        int i = 1;
        for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
            DPoint Pu = *iter;
            node u = G.newNode();
            GA.x(u) = Pu.m_x;
            GA.y(u) = Pu.m_y;
            GA.idNode(u) =  i++;
        }

        // Add graph to planner
        m_planner.addWaypoints(G, GA);
    }

protected:
    dpp::DubinsVehiclePathPlanner m_planner;
    std::string m_filename;

}; // DubinsVehiclePathPlannerFeatureTest


// Test for SetUp worked
TEST_F(DubinsVehiclePathPlannerFeatureTest, SetUpWorked) {
    ASSERT_EQ(0, m_planner.waypointCount());
    ASSERT_EQ(0, m_planner.haveSolution());

    EXPECT_DOUBLE_EQ(INITIAL_HEADING, m_planner.initialHeading());
    EXPECT_DOUBLE_EQ(TURN_RADIUS, m_planner.turnRadius());
}

// Test for setting planning algorithm
TEST_F(DubinsVehiclePathPlannerFeatureTest, SettingDtspPlanningAlgorithm) {
    // Test for alternating algorithm
    DtspPlanningAlgorithm planAlg = DtspPlanningAlgorithm::ALTERNATING;
    m_planner.algorithm(planAlg);
    EXPECT_STREQ("Alternating", m_planner.algorithmName().c_str());

    // Test for nearest neighbor algorithm
    planAlg = DtspPlanningAlgorithm::NEAREST_NEIGHBOR;
    m_planner.algorithm(planAlg);
    EXPECT_STREQ("NearestNeighbor", m_planner.algorithmName().c_str());

    // Test for randomized algorithm
    planAlg = DtspPlanningAlgorithm::RANDOMIZED;
    m_planner.algorithm(planAlg);
    EXPECT_STREQ("Randomized", m_planner.algorithmName().c_str());

    // Test for death with unknown algorithm
    planAlg = static_cast<DtspPlanningAlgorithm>(
        std::numeric_limits<int>::max());
    EXPECT_DEATH(m_planner.algorithm(planAlg), "Assertion failed: \\(0 && \"Unknown planning algorithm\"\\).*");
}

const List<DPoint> nodes {
    {0, 0},
    {5.1*TURN_RADIUS, 0.0},
    {5.1*TURN_RADIUS, -3.3*TURN_RADIUS}
};

// Test for adding waypoints from a graph
TEST_F(DubinsVehiclePathPlannerFeatureTest, WaypointsFromGraph) {
    //std::cout << "planner.m_G: " << m_planner.graphPtr() << std::endl;
    //dpp::Logger *log = dpp::Logger::Instance();
    //log->verbose(1);

    // Add the nodes to the planner
    initializeWaypoints(nodes);

    NodeArray<double> Headings = m_planner.headings();
    Graph *pG = m_planner.graphPtr();
    GraphAttributes GA = m_planner.graphAttributes();

    // Verify that members point to the same graph
    EXPECT_EQ(pG, Headings.graphOf());
    EXPECT_EQ(pG, &(GA.constGraph()));

    // Verify that points were set
    EXPECT_EQ(nodes.size(), m_planner.waypointCount());

    node u = pG->firstNode();
    ListConstIterator<DPoint> iter;
    int i = 1;
    for ( iter = nodes.begin(); (u) && iter != nodes.end();
        iter++, (u)=(u)->succ() ) {
        DPoint Pu = *iter;
        EXPECT_DOUBLE_EQ(Pu.m_x, GA.x(u));
        EXPECT_DOUBLE_EQ(Pu.m_y, GA.y(u));
        EXPECT_EQ(i, GA.idNode(u));
        i++;
    }
    EXPECT_EQ(nodes.size(), i - 1);
}

// TODO test for adding waypoints from file
// TOOD test for each algorithm
// TODO add test for copySolution()
// Test for copySolution() produces an equivalent graph.
TEST_F(DubinsVehiclePathPlannerFeatureTest, CopySolutionEquivalence) {
    // Add the nodes to the planner
    initializeWaypoints(nodes);

    EXPECT_TRUE(m_planner.solve());
    EXPECT_TRUE(m_planner.haveSolution());

    Graph G;
    GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES);
    NodeArray<double> Headings(G);
    List<node> Tour;
    List<edge> Edges;
    double cost;

    m_planner.copySolution(G, GA, Tour, Edges, Headings, cost);

    Graph m_G = *(m_planner.graphPtr());
    EXPECT_TRUE(dpp::graphsAreEquivalent(*(m_planner.graphPtr()),
        m_planner.graphAttributes(), G, GA));

    // TODO compare tour, edges, and headings

}
