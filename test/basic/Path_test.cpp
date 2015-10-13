/*
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <DubinsCurves.h>

#include <dpp/basic/Logger.h>
#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/basic/Path.h>
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

using std::vector;

using dpp::VehicleConfiguration;

#define TURN_RADIUS     1.0
#define EPSILON_ERROR   0.05 // error margin in length calculation

// Test for dubinsPathLength()
TEST(DubinsPathLengthTest, FailsBelowMinDistance) {
    VehicleConfiguration c1(0.0,0.0,0.0), c2(0.0,0.0,0.0);
    EXPECT_DEATH(dpp::dubinsPathLength(c1, c2, TURN_RADIUS), "Assertion failed: \\(dist >= 3\\.0 \\* r\\)");
}

// Test against DubinsCurves lib
TEST(DubinsPathLengthTest, RSR) {
    VehicleConfiguration c1(0.0,0.0,0.0),
        c2(5.0*TURN_RADIUS,0, dpp::degToRad(180.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = dpp::headingToAngle(q0[2]);
    q1[2] = dpp::headingToAngle(q1[2]);
    DubinsCurves::DubinsPath path;
    DubinsCurves::dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = DubinsCurves::dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dpp::dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, RSL) {
    VehicleConfiguration c1(0.0,0.0,dpp::degToRad(37)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, dpp::degToRad(340.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = dpp::headingToAngle(q0[2]);
    q1[2] = dpp::headingToAngle(q1[2]);
    DubinsCurves::DubinsPath path;
    DubinsCurves::dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = DubinsCurves::dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dpp::dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, LSR) {
    VehicleConfiguration c1(0.0,0.0,dpp::degToRad(245)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, dpp::degToRad(165.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = dpp::headingToAngle(q0[2]);
    q1[2] = dpp::headingToAngle(q1[2]);
    DubinsCurves::DubinsPath path;
    DubinsCurves::dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = DubinsCurves::dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dpp::dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, LSL) {
    VehicleConfiguration c1(0.0,0.0, dpp::degToRad(170)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, dpp::degToRad(315.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = dpp::headingToAngle(q0[2]);
    q1[2] = dpp::headingToAngle(q1[2]);
    DubinsCurves::DubinsPath path;
    DubinsCurves::dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = DubinsCurves::dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dpp::dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, SimilarHeadings) {
    Vector2d u(450.0, 0.0),
        v(350.0, -100.0);
    double X = dpp::headingBetween(u,v);
    VehicleConfiguration c1(450.0, 0.0, X),
        c2(350.0, -100.0, X);
            // 5pi/4 = 3.9269908169872413949974543 ~ 5.0*M_PI/4.0

    double expectedLength = 141.4213562373095;

    EXPECT_DOUBLE_EQ(expectedLength, dpp::dubinsPathLength(c1,c2,TURN_RADIUS));
}

// Value-parameterized tests for dubinsTourCost()
//#if GTEST_HAS_PARAM_TEST
using ::testing::Test;
//using ::testing::TestWithParam;
using ::testing::Values;
using ::testing::ValuesIn;

// Some node configurations to use
const List<VehicleConfiguration> nodeConfigs { 
    {0.0, 0.0, 0.0}, 
    {5.1*TURN_RADIUS, 0.0, dpp::degToRad(180)}, 
    {5.1*TURN_RADIUS, -3.3*TURN_RADIUS, dpp::degToRad(270)}
}; 

class DubinsTourTest : public Test {
public:
    DubinsTourTest()
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
          m_nodeHeadings(m_G) {}

    virtual ~DubinsTourTest() { }

    //void InitializeScenario(List<VehicleConfiguration> configs) {
    virtual void SetUp() {
        m_n = nodeConfigs.size();
        int i = 1; // node index
        //m_nodeList = new node*[m_n];
        m_nodeList.resize(m_n);

        // Build graph
        ListConstIterator<VehicleConfiguration> iter;
        
        for ( iter = nodeConfigs.begin(); iter != nodeConfigs.end(); iter++ ) {
            VehicleConfiguration Cu = *iter;
            node u = m_G.newNode();
            m_GA.x(u) = Cu.x();
            m_GA.y(u) = Cu.y();
            m_GA.idNode(u) =  i;

            m_nodeList[i-1] = u; // [i - 1] = &u;
            //std::cout << "Added u@" << (u) << " at i=" << (i-1) << "." << std::endl;
            m_nodeHeadings(u) = Cu.heading();
            i++;
        }

    } // InitializeScenario()

    virtual void TearDown() {
    } // TearDown()

    int GetIndex(node &u) {
        return m_GA.idNode(u);
    }

    node GetNode(int i) {
        i -= 1;
        if (i < m_n) {
            //std::cout << "Returning i=" << i << "." << std::endl;
            return m_nodeList.at(i);
        }
        return nullptr;
    }

    int GetSize() {
        return m_n;
    }

protected:
    Graph m_G;
    GraphAttributes m_GA;
    NodeArray<double> m_nodeHeadings;
    vector<node> m_nodeList;
    int m_n;

}; // class DubinsTourTest 

// Tests for dubinsTourLength
TEST_F(DubinsTourTest, EmptyTourHasZeroCost) {
    //InitializeScenario(noConfigs);

    // Check that empty tours have zero cost regardless of whether returnCost is set or not
    List<node> tour;
    double expectedTourCost = 0.0;

    EXPECT_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, false));
    EXPECT_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, true));
}

TEST_F(DubinsTourTest, ShortTourHasZeroCost) {
    ASSERT_GT(GetSize(), 0);
    List<node> tour;

    // Check that single node tours have zero cost ---------------"-------------
    tour.pushBack(GetNode(1));
    double expectedTourCost = 0.0;

    EXPECT_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, false));
    EXPECT_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, true));
}

TEST_F(DubinsTourTest, TourIgnoresReturn) {
    List<node> tour;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));
    double expectedTourCost = 6.241592653589793;

    // Two-node tour works. no returns
    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));
    
    // Same cost, because function ignores returning to node 1
    tour.pushBack(GetNode(1));
    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));
}

TEST_F(DubinsTourTest, TourFollowsReturn) {
    List<node> tour;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));
    double expectedTourCost = 12.483185307179586;

    // Two-node tour returns
    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, true));

    // Same cost, because function will not add return node 1 to the end
    tour.pushBack(GetNode(1));
    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, true));
}


TEST_F(DubinsTourTest, TourWithThreeNodes) {
    //InitializeScenario(nodeConfigs);
    ASSERT_EQ(3, GetSize());
    double expectedTourCost = 10.351530620781304;
    double expectedReturnTourCost = 16.623390656993465;

    List<node> tour;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));
    tour.pushBack(GetNode(3));

    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));

    EXPECT_DOUBLE_EQ(expectedReturnTourCost, dpp::dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, true));

}

class DubinsTourEdgesTest : public DubinsTourTest {
};


// Tests for createDubinsTourEdges
TEST_F(DubinsTourEdgesTest, FailsWithExistingEdges) {
    List<node> tour;
    List<edge> edges;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));

    edge e = m_G.newEdge(GetNode(1), GetNode(2));
    EXPECT_EQ(1, m_G.numberOfEdges());

    EXPECT_DEATH(dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, false), "Assertion failed: \\(G\\.numberOfEdges\\(\\) < 1\\)");
    EXPECT_DEATH(dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, true), "Assertion failed: \\(G\\.numberOfEdges\\(\\) < 1\\)");
} // FailsWithExistingEdges

TEST_F(DubinsTourEdgesTest, EmptyTourHasNoEdges) {
    //InitializeScenario(noConfigs);
    List<node> tour;
    List<edge> edges;
    double expectedTourCost = 0.0;

    // No cost, no edges
    EXPECT_EQ(expectedTourCost, dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, true));
    EXPECT_EQ(0, m_G.numberOfEdges());
    EXPECT_EQ(0, edges.size());

    // Same with return edge option set
    dpp::clearEdges(m_G);
    EXPECT_EQ(expectedTourCost, dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, true));
    EXPECT_EQ(0, m_G.numberOfEdges());
    EXPECT_EQ(0, edges.size());
} // EmptyTourHasNoEdges


TEST_F(DubinsTourEdgesTest, ShortTourHasNoEdges) {
    ASSERT_GT(GetSize(), 0);
    List<node> tour;
    List<edge> edges;
    tour.pushBack(GetNode(1));

    // No cost, no edges
    EXPECT_EQ(0.0, dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, true));
    EXPECT_EQ(0, m_G.numberOfEdges());
    EXPECT_EQ(0, edges.size());

    // Same with return edge option set
    dpp::clearEdges(m_G);
    EXPECT_EQ(0.0, dpp::createDubinsTourEdges(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS,
        edges, true));
    EXPECT_EQ(0, m_G.numberOfEdges());
    EXPECT_EQ(0, edges.size());
} // ShortTourHasNoEdges

TEST_F(DubinsTourEdgesTest, NoReturnEdge) {
    List<node> tour;
    List<edge> edges;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));

    // Cost matches, and we have 1 edge
    double expected_cost = 6.241592653589793;
    EXPECT_DOUBLE_EQ(expected_cost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, false));
    EXPECT_EQ(1, m_G.numberOfEdges());
    EXPECT_EQ(1, edges.size());
    edge e = edges.front();
    EXPECT_EQ(GetNode(1), e->source());
    EXPECT_EQ(GetNode(2), e->target());
    EXPECT_EQ(expected_cost, m_GA.doubleWeight(e));
    
    // Same with no return edge (ignore extra node in tour)
    dpp::clearEdges(m_G);
    edges.clear();
    tour.pushBack(GetNode(1));
    EXPECT_DOUBLE_EQ(expected_cost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, false));
    EXPECT_EQ(1, m_G.numberOfEdges());
    EXPECT_EQ(1, edges.size());
    e = edges.front();
    EXPECT_EQ(GetNode(1), e->source());
    EXPECT_EQ(GetNode(2), e->target());
    EXPECT_EQ(expected_cost, m_GA.doubleWeight(e));
} // NoReturnEdge

TEST_F(DubinsTourEdgesTest, ReturnEdge) {
    List<node> tour;
    List<edge> edges;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));

    // Cost matches, and we have 1 edge
    double expected_cost = 12.483185307179586;
    EXPECT_DOUBLE_EQ(expected_cost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, true));
    EXPECT_EQ(2, m_G.numberOfEdges());
    EXPECT_EQ(2, edges.size());
    edge e1 = edges.front();
    edge e2 = edges.front()->succ();
    EXPECT_EQ(GetNode(1), e1->source());
    EXPECT_EQ(GetNode(2), e1->target());
    EXPECT_EQ(6.241592653589793, m_GA.doubleWeight(e1));
    EXPECT_EQ(GetNode(2), e2->source());
    EXPECT_EQ(GetNode(1), e2->target());
    EXPECT_EQ(6.241592653589793, m_GA.doubleWeight(e2));
    
    // Same with no return edge (ignore extra node in tour)
    dpp::clearEdges(m_G);
    edges.clear();
    tour.pushBack(GetNode(1));
    EXPECT_DOUBLE_EQ(expected_cost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, true));
    EXPECT_EQ(2, m_G.numberOfEdges());
    EXPECT_EQ(2, edges.size());
    e1 = edges.front();
    e2 = edges.front()->succ();
    EXPECT_EQ(GetNode(1), e1->source());
    EXPECT_EQ(GetNode(2), e1->target());
    EXPECT_EQ(6.241592653589793, m_GA.doubleWeight(e1));
    EXPECT_EQ(GetNode(2), e2->source());
    EXPECT_EQ(GetNode(1), e2->target());
    EXPECT_EQ(6.241592653589793, m_GA.doubleWeight(e2));
} // ReturnEdge

TEST_F(DubinsTourEdgesTest, TourWithThreeNodes) {
    //InitializeScenario(nodeConfigs);
    ASSERT_EQ(3, GetSize());

/*
    dpp::Logger *log = dpp::Logger::Instance();
    log->verbose(DPP_LOGGER_VERBOSE_2);
    log->level(dpp::Logger::Level::LL_DEBUG);
    */

    List<node> tour;
    List<edge> edges;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));
    tour.pushBack(GetNode(3));
    double expectedTourCost = 10.351530620781304;
    double expectedReturnTourCost = 16.623390656993465;

    EXPECT_DOUBLE_EQ(expectedTourCost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, false));

    dpp::clearEdges(m_G);
    edges.clear();
    EXPECT_DOUBLE_EQ(expectedReturnTourCost, dpp::createDubinsTourEdges(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, edges, true));
}


/* This is for testing different data on the same tests. 
 * Don't need it yet! */
/*
INSTANTIATE_TEST_CASE_P(Predetermined,
    DubinsTourTest,
    Values(nodeConfigs));
*/

//#else

// Google Test may not support value-parameterized tests with some
// compilers. This ifdef skirts around LNK1561 fatal error on Windows.
//TEST(DummyTest, ValueParameterizedTestsAreNotSupportedOnThisPlatform) {}

//#endif  // GTEST_HAS_PARAM_TEST
