/*
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <iostream>
#include <vector>
#include <math.h>

#include <gtest/gtest.h>

#include <dpp/basic/Logger.h>
#include <dpp/planalg/RandomizedDTSP.h>
#include <dpp/basic/Util.h>

using ogdf::node;
using ogdf::edge;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::DPoint;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::ListConstIterator;
using ogdf::NodeArray;

#define TURN_RADIUS     1.0
#define EPSILON_ERROR   0.05 // error margin in length calculation
#define EPSILON_RANDOM_COST (5.0*TURN_RADIUS) // stddev of solution cost


// Value-parameterized tests for dubinsTourCost()
//#if GTEST_HAS_PARAM_TEST
using ::testing::Test;
//using ::testing::TestWithParam;
using ::testing::Values;
using ::testing::ValuesIn;

const List<DPoint> nodes {
    {0, 0},
    {5.1*TURN_RADIUS, 0.0},
    {5.1*TURN_RADIUS, -3.3*TURN_RADIUS},
    {0, -6.6*TURN_RADIUS},
    {5.1*TURN_RADIUS, -6.6*TURN_RADIUS}
};

class RandomizedDTSPTest : public Test {
public:
    RandomizedDTSPTest()
        : m_G(),
          m_GA (m_G, 
            GraphAttributes::nodeGraphics | 
            GraphAttributes::edgeGraphics |
            GraphAttributes::nodeLabel |
            GraphAttributes::edgeStyle |
            GraphAttributes::edgeDoubleWeight |
            GraphAttributes::nodeStyle |
            GraphAttributes::nodeTemplate |
            GraphAttributes::nodeId),
          m_Headings(m_G),
          m_turnRadius(TURN_RADIUS),
          m_initialHeading(0.0),
          m_alg()
    { }

    virtual ~RandomizedDTSPTest() { }

    virtual void SetUp() {
        int m = nodes.size();

        // Build graph
        ListConstIterator<DPoint> iter;
        int i = 1;
        for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
            DPoint Pu = *iter;
            node u = m_G.newNode();
            m_GA.x(u) = Pu.m_x;
            m_GA.y(u) = Pu.m_y;
            m_GA.idNode(u) =  i++;
        }

    } // InitializeScenario()

    virtual void TearDown() {
    } // TearDown()

    int GetSize() {
        return m_G.numberOfNodes();
    }

protected:
    Graph m_G;
    GraphAttributes m_GA;
    NodeArray<double> m_Headings;
    List<node> m_Tour;
    List<edge> m_Edges;
    double m_cost;
    const double m_turnRadius;
    double m_initialHeading;
    dpp::RandomizedDTSP m_alg;

}; // class RandomizedDTSPTest 

// Test for out of range error
TEST_F(RandomizedDTSPTest, InitialHeadingOutOfRangeError) {
    double x = -1.1; // initial heading

    EXPECT_THROW(m_alg.run(m_G, m_GA, x, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost), std::out_of_range);

    x = 2*M_PI; // initial heading

    EXPECT_THROW(m_alg.run(m_G, m_GA, x, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost), std::out_of_range);
}

// Test for headings domain error
TEST_F(RandomizedDTSPTest, HeadingsDomainError) {
    Graph G2;
    m_Headings.init(G2);
    EXPECT_THROW(m_alg.run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost), std::domain_error);
}

// Test for not enough nodes error
TEST_F(RandomizedDTSPTest, NodesOutOfRangeError) {
    node u;
    // This doesn't work? 
    //std::cout << "Graph: " << &m_G << std::endl;
    //forall_nodes(u,m_G) {
    //    std::cout << "Node " << m_GA.idNode(u) << " of graph: " << u->graphOf() << std::endl;
    //    m_G.delNode(u);
    //}
    m_G.clear();

    EXPECT_THROW(m_alg.run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost), std::out_of_range);
}

// Test for assertion fails with existing edges
TEST_F(RandomizedDTSPTest, ExistingEdgesFails) {
    // shouldn't need to add this to m_Edges
    edge e = m_G.newEdge(m_G.firstNode(), m_G.lastNode()); 

    EXPECT_DEATH(m_alg.run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost), "Assertion failed: \\(G\\.numberOfEdges\\(\\) < 1\\).*");
}

// Test for working tour with no return
TEST_F(RandomizedDTSPTest, TourWithNoReturn) {
    double expectedCost = 20.15;
    int expectedEdgesSize = GetSize() - 1;
    int expectedTourSize = GetSize();

    EXPECT_EQ(SUCCESS, m_alg.run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost, false));
    EXPECT_EQ(expectedTourSize, m_Tour.size());
    EXPECT_EQ(expectedEdgesSize, m_Edges.size());
    EXPECT_NEAR(expectedCost, m_cost, EPSILON_RANDOM_COST);
}

// Test for working tour with return
TEST_F(RandomizedDTSPTest, TourWithReturn) {
    double expectedCost = 29.85;
    int expectedEdgesSize = GetSize();
    int expectedTourSize = GetSize() + 1;

    dpp::Logger *log = dpp::Logger::Instance();
    log->level(dpp::Logger::Level::LL_DEBUG);
    log->verbose(0);

    EXPECT_EQ(SUCCESS, m_alg.run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour, m_Edges,
        m_Headings, m_cost, true));
    EXPECT_EQ(expectedTourSize, m_Tour.size());
    EXPECT_EQ(expectedEdgesSize, m_Edges.size());
    EXPECT_NEAR(expectedCost, m_cost, EPSILON_RANDOM_COST);
}
