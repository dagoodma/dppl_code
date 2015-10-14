/*
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <math.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <ogdf/basic/List.h>

#include <dpp/basic/Util.h>

using Eigen::Vector3d;
using Eigen::Vector2d;

using ogdf::List;
using ogdf::ListIterator;
using ogdf::ListConstIterator;
using ogdf::NodeArray;
using ogdf::DPoint;

using std::vector;

// TODO parameterize data-driven tests in this file

#define DELTA           1E-2 // input perturbation constant
#define EPSILON_ERROR   1E-5  // output error threshold

// --------------------- myMod() -------------------
TEST(MyModTest, FmodEquivalenceComparison) {
    EXPECT_DOUBLE_EQ(fmod(0.0,2), dpp::myMod(0.0,2));
    EXPECT_DOUBLE_EQ(fmod(1.0,2), dpp::myMod(1.0,2));
    EXPECT_DOUBLE_EQ(fmod(1.0,2), dpp::myMod(1.0,2));
    EXPECT_DOUBLE_EQ(fmod(2.0,2), dpp::myMod(2.0,2));
    EXPECT_DOUBLE_EQ(fmod(2.1,2), dpp::myMod(2.1,2));
    EXPECT_DOUBLE_EQ(fmod(5,2), dpp::myMod(5,2));
}

TEST(MyModTest, RemovesNegatives) {
    EXPECT_DOUBLE_EQ(1, dpp::myMod(-1.0,2));
    EXPECT_DOUBLE_EQ(0, dpp::myMod(-2.0,2));
    EXPECT_DOUBLE_EQ(1.9, dpp::myMod(-2.1,2));
}

// --------------------- headingToAngle() -------------------
TEST(HeadingToAngleTest, Quadrants) {
    EXPECT_DOUBLE_EQ(M_PI/2.0, dpp::headingToAngle(0.0));
    EXPECT_DOUBLE_EQ(0.0, dpp::headingToAngle(M_PI/2.0));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, dpp::headingToAngle(M_PI));
    EXPECT_DOUBLE_EQ(M_PI, dpp::headingToAngle(3.0*M_PI/2.0));
}

// --------------------- wrapAngle() -------------------
TEST(WrapAngleTest, NoWrap) {
    EXPECT_DOUBLE_EQ(0.0, dpp::wrapAngle(0.0));
    EXPECT_DOUBLE_EQ(M_PI, dpp::wrapAngle(M_PI));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, dpp::wrapAngle(3.0*M_PI/2.0));
}
TEST(WrapAngleTest, Wrap) {
    EXPECT_DOUBLE_EQ(0.0, dpp::wrapAngle(2.0*M_PI));
    EXPECT_DOUBLE_EQ(M_PI/2.0, dpp::wrapAngle(5.0*M_PI/2.0));
    EXPECT_NEAR(DELTA, dpp::wrapAngle(2.0*M_PI + DELTA), EPSILON_ERROR);
}
TEST(WrapAngleTest, TwoPiWraps) {
    double input=2*M_PI;
    double expected_output=0.0;
    EXPECT_NEAR(expected_output, dpp::wrapAngle(input), EPSILON_ERROR);
}
TEST(WrapAngleTest, BoundaryWrap) {
    double input=6.283195307179586;
    double expected_output=0.0;
    EXPECT_NEAR(expected_output, dpp::wrapAngle(input), EPSILON_ERROR);
}

// --------------------- headingBetween() -------------------
TEST(HeadingBetweenTest, atan) {
    double x1=0,y1=0,x2=1,y2=0;
    EXPECT_DOUBLE_EQ(0.0, atan((y1 - y2)/(x2 - x1)));
}

const Vector2d u(0.0, 0.0);
TEST(HeadingBetweenTest, QuadrantI) {
    // Test Case for Quadrant 1 Boundaries
    Vector2d v(
        0.0,
        1.0);
    EXPECT_DOUBLE_EQ(0.0, dpp::headingBetween(u, v));

    v[0] = 1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(dpp::degToRad(45.0), dpp::headingBetween(u,v));
}

TEST(HeadingBetweenTest, QuadrantII) {
    // Test Case for Quadrant 2 Boundaries
    Vector2d v(
        1.0,
        0.0);
    EXPECT_DOUBLE_EQ(dpp::degToRad(90.0), dpp::headingBetween(u,v));

    v[0] = 1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(dpp::degToRad(135.0), dpp::headingBetween(u,v));
}
TEST(HeadingBetweenTest, QuadrantIII) {
    // Test Case for Quadrant 3 Boundaries
    Vector2d v(
        0.0,
        -1.0);
    EXPECT_DOUBLE_EQ(dpp::degToRad(180.0), dpp::headingBetween(u,v));

    v[0] = -1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(dpp::degToRad(225.0), dpp::headingBetween(u,v));
}

TEST(HeadingBetweenTest, QuadrantIV) {
    // Test Case for Quadrant 4 Boundaries
    Vector2d v(
        -1.0,
        0.0);
    EXPECT_DOUBLE_EQ(dpp::degToRad(270.0), dpp::headingBetween(u,v));

    v[0] = -1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(dpp::degToRad(315.0), dpp::headingBetween(u,v));
}

TEST(HeadingBetweenTest, Various) {
    // Some various test cases
    Vector2d u(
        1.0,
        1.0),
        v(
        1.0,
        -1.0);
    EXPECT_DOUBLE_EQ(dpp::degToRad(180), dpp::headingBetween(u,v));
    EXPECT_DOUBLE_EQ(dpp::degToRad(0), dpp::headingBetween(v,u));
}

// --------------------- clearEdges() -------------------
using ogdf::node;
using ogdf::edge;
using ogdf::Graph;
using ogdf::GraphAttributes;

using ::testing::Test;
using ::testing::Values;
using ::testing::ValuesIn;
class ClearEdgesTest : public Test {
public:
    ClearEdgesTest()
        : m_G(),
          m_GA (m_G, 
            GraphAttributes::nodeId) {}

    virtual ~ClearEdgesTest() { }

    virtual void SetUp() {
        // Add nodes
        node u = m_G.newNode();
        node v = m_G.newNode();
        m_nodeList.push_back(u);
        m_nodeList.push_back(v);

    } // InitializeScenario()

    virtual void TearDown() {
    } // TearDown()

    int GetIndex(node &u) {
        return m_GA.idNode(u);
    }

    node GetNode(int i) {
        i -= 1;
        if (i < m_G.numberOfNodes()) {
            return m_nodeList.at(i);
        }
        return nullptr;
    }

    int GetSize() {
        return m_G.numberOfEdges();
    }

protected:
    Graph m_G;
    GraphAttributes m_GA;
    vector<node> m_nodeList;

}; // class ClearEdgesTest 

TEST_F(ClearEdgesTest, NoEdges) {
    // Nothing is cleared
    EXPECT_EQ(0, GetSize());
    dpp::clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}

TEST_F(ClearEdgesTest, OneEdge) {
    EXPECT_EQ(0, GetSize());

    // Add an edge
    edge e = m_G.newEdge(GetNode(1), GetNode(2));
    EXPECT_EQ(1, GetSize());

    // Delete it
    dpp::clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}


TEST_F(ClearEdgesTest, TwoEdges) {
    EXPECT_EQ(0, GetSize());

    // Add two edges
    edge e1 = m_G.newEdge(GetNode(1), GetNode(2));
    edge e2 = m_G.newEdge(GetNode(2), GetNode(1));
    EXPECT_EQ(2, GetSize());

    // Delete them
    dpp::clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}

// --------------------- graphsAreEquivalent() -------------------

// These nodes are used in several tests
const List<DPoint> nodes {
    {0, 0},
    {5.1, 0.0},
    {5.1, -3.3}
};

class GraphsAreEquivalentTest : public Test {
public:
    GraphsAreEquivalentTest()
        : m_G(),
          m_GA (m_G, DPP_GRAPH_ATTRIBUTES ),
          m_Gcopy(),
          m_GAcopy(m_Gcopy, DPP_GRAPH_ATTRIBUTES)
    { }

    virtual ~GraphsAreEquivalentTest() { }

    virtual void SetUp() {
        // Add nodes to original
        int i = 1;
        ListConstIterator<DPoint> iter;
        for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
            DPoint Pu = *iter;
            node u = m_G.newNode();
            m_GA.x(u) = Pu.m_x;
            m_GA.y(u) = Pu.m_y;
            m_GA.idNode(u) = i;
            i++;
        }

        // Add the same nodes to copy
        int icopy = 1;
        for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
            DPoint Pu = *iter;
            node u = m_Gcopy.newNode();
            m_GAcopy.x(u) = Pu.m_x;
            m_GAcopy.y(u) = Pu.m_y;
            m_GAcopy.idNode(u) = icopy;
            icopy++;
        }

        // Generate random seed for edge weight generation
        srand (static_cast <unsigned> (time(0)));
    } // SetUp()

    virtual void TearDown() {
    } // TearDown()

    // Returns a random from 0 to 1.0
    double getRandomEdgeWeight(void) {
        return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }

    // Add directed edges between nodes in index order with random weights
    void addEdges() {
        node u, v;
        forall_nodes(v,m_G) {
            // Skip the first node
            if (m_G.firstNode() == v) {
                u = v;
                continue;
            }
            edge e = m_G.newEdge(u, v);
            // Generate: w = [0, 1.0]
            float w = getRandomEdgeWeight();
            m_GA.doubleWeight(e) = w;
            u = v;
        }
        forall_nodes(v,m_Gcopy) {
            // Skip the first node
            if (m_Gcopy.firstNode() == v) {
                u = v;
                continue;
            }
            edge e = m_Gcopy.newEdge(u, v);
            // Generate: w = [0, 1.0]
            float w = getRandomEdgeWeight();
            m_GAcopy.doubleWeight(e) = w;
            u = v;
        }
    }

protected:
    Graph m_G, m_Gcopy;
    GraphAttributes m_GA, m_GAcopy;

}; // class GraphsAreEquivalentTest 

// Test for equivalence with empty graphs
TEST_F(GraphsAreEquivalentTest, EmptyGraphsAreEquivalent) {
    Graph G, Gcopy;
    GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES),
        GAcopy(Gcopy, DPP_GRAPH_ATTRIBUTES);

    ASSERT_EQ(0, G.numberOfNodes());
    ASSERT_EQ(0, Gcopy.numberOfNodes());
    ASSERT_EQ(0, G.numberOfEdges());
    ASSERT_EQ(0, Gcopy.numberOfEdges());

    EXPECT_EQ(1, dpp::graphsAreEquivalent(G, GA, Gcopy, GAcopy));
}

// Test for equivalence with empty graphs
TEST_F(GraphsAreEquivalentTest, SimpleGraphsNotEquivalent) {
    Graph G, Gcopy;
    GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES),
        GAcopy(Gcopy, DPP_GRAPH_ATTRIBUTES);

    node u = G.newNode();
    GA.x(u) = getRandomEdgeWeight();
    GA.x(u) = getRandomEdgeWeight();
    GA.idNode(u) = 1;

    ASSERT_EQ(1, G.numberOfNodes());
    ASSERT_EQ(0, Gcopy.numberOfNodes());
    ASSERT_EQ(0, G.numberOfEdges());
    ASSERT_EQ(0, Gcopy.numberOfEdges());

    EXPECT_EQ(0, dpp::graphsAreEquivalent(G, GA, Gcopy, GAcopy));
}

// Test for equivalence with node-only graphs
TEST_F(GraphsAreEquivalentTest, NodeOnlyGraphsAreEquivalent) {
    int expectedSize = nodes.size();
    ASSERT_EQ(expectedSize, m_G.numberOfNodes());
    ASSERT_EQ(expectedSize, m_Gcopy.numberOfNodes());

    EXPECT_EQ(1, dpp::graphsAreEquivalent(m_G, m_GA, m_Gcopy, m_GAcopy));
}

// Test for equivalence with full graphs
TEST_F(GraphsAreEquivalentTest, GraphsAreEquivalent) {
    int expectedSize = nodes.size();
    ASSERT_EQ(expectedSize, m_G.numberOfNodes());
    ASSERT_EQ(expectedSize, m_Gcopy.numberOfNodes());

    addEdges();
    ASSERT_EQ(m_G.numberOfNodes() - 1, m_G.numberOfEdges());
    ASSERT_EQ(m_G.numberOfNodes() - 1, m_Gcopy.numberOfEdges());

    EXPECT_EQ(1, dpp::graphsAreEquivalent(m_G, m_GA, m_Gcopy, m_GAcopy));
}

// --------------------- copyGraph() -------------------
class GraphCopyTest : public Test {
public:
    GraphCopyTest()
        : m_G(),
          m_GA (m_G, DPP_GRAPH_ATTRIBUTES ),
          m_Gcopy(),
          m_GAcopy (m_Gcopy, DPP_GRAPH_ATTRIBUTES )
    { }

    virtual ~GraphCopyTest() { }

    virtual void SetUp() {
        // Add nodes
        int i = 1;
        ListConstIterator<DPoint> iter;
        for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
            DPoint Pu = *iter;
            node u = m_G.newNode();
            m_GA.x(u) = Pu.m_x;
            m_GA.y(u) = Pu.m_y;
            m_GA.idNode(u) = i;
            i++;
        }
        // Generate random seed for edge weight generation
        srand (static_cast <unsigned> (time(0)));
    } // SetUp()

    virtual void TearDown() {
    } // TearDown()

    // Add directed edges between nodes in index order with random weights
    void addEdges() {
        node u, v;
        forall_nodes(v,m_G) {
            // Skip the first node
            if (m_G.firstNode() == v) {
                u = v;
                continue;
            }
            edge e = m_G.newEdge(u, v);
            float w = getRandomEdgeWeight();
            m_GA.doubleWeight(e) = w;
        }
    }

    // Returns a random from 0 to 1.0
    double getRandomEdgeWeight(void) {
        return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }

protected:
    Graph m_G, m_Gcopy;
    GraphAttributes m_GA, m_GAcopy;

}; // class GraphCopyTest 

TEST_F(GraphCopyTest, CopiesNodesOnly) {
    ASSERT_GT(m_G.numberOfNodes(), 0);
    ASSERT_EQ(m_G.numberOfEdges(), 0);
    ASSERT_EQ(m_Gcopy.numberOfNodes(), 0);
    ASSERT_EQ(m_Gcopy.numberOfEdges(), 0);

    int n = dpp::copyGraph(m_G, m_GA, m_Gcopy, m_GAcopy);

    EXPECT_EQ(m_G.numberOfNodes(), n);

    EXPECT_EQ(true, dpp::graphsAreEquivalent(m_G, m_GA, m_Gcopy, m_GAcopy)); 
}

TEST_F(GraphCopyTest, ClearsExistingNodes) {
    ASSERT_GT(m_G.numberOfNodes(), 0);
    ASSERT_EQ(m_G.numberOfEdges(), 0);
    ASSERT_EQ(m_Gcopy.numberOfNodes(), 0);
    ASSERT_EQ(m_Gcopy.numberOfEdges(), 0);

    node u = m_Gcopy.newNode();
    m_GAcopy.x(u) = 0.5;
    m_GAcopy.y(u) = 0.77;
    m_GAcopy.idNode(u) = 1;

    ASSERT_EQ(1, m_Gcopy.numberOfNodes());

    int n = dpp::copyGraph(m_G, m_GA, m_Gcopy, m_GAcopy);

    EXPECT_EQ(m_G.numberOfNodes(), n);

    EXPECT_EQ(true, dpp::graphsAreEquivalent(m_G, m_GA, m_Gcopy, m_GAcopy)); 
}
/*
TEST_F(GraphCopyTest, CopiesNodesAndEdges) {
    ASSERT_GT(m_G.numberOfNodes(), 0);
    ASSERT_EQ(m_G.numberOfEdges(), 0);

    addEdges();
    ASSERT_EQ(m_G.numberOfNodes() - 1, m_G.numberOfEdges());

    int n = dpp::copyGraph(m_G, m_GA, Gcopy, GAcopy);

    EXPECT_EQ(m_G.numberOfNodes(), n);

    EXPECT_EQ(true, dpp::graphsAreEquivalent(m_G, m_GA, m_Gcopy, m_GAcopy)); 
}
*/