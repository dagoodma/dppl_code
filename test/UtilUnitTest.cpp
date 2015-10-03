/*
 * Copyright (C) 2014-2015 DubinsAreaCoverage.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the COPYRIGHT file distributed with DubinsAreaCoverage.
*/
#include <math.h>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "Util.h"

using Eigen::Vector3d;
using Eigen::Vector2d;

using ogdf::List;
using ogdf::ListIterator;
using ogdf::ListConstIterator;
using ogdf::NodeArray;

using std::vector;


#define DELTA           1E-2 // for testing inputs 
#define EPSILON_ERROR   1E-5  // Close enough for government work

// TODO parameterize data-driven tests in this file

// --------------------- headingToAngle() -------------------
TEST(HeadingToAngleTest, Quadrants) {
    EXPECT_DOUBLE_EQ(M_PI/2.0, headingToAngle(0.0));
    EXPECT_DOUBLE_EQ(0.0, headingToAngle(M_PI/2.0));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, headingToAngle(M_PI));
    EXPECT_DOUBLE_EQ(M_PI, headingToAngle(3.0*M_PI/2.0));
}

// --------------------- wrapAngle() -------------------
TEST(WrapAngleTest, NoWrap) {
    EXPECT_DOUBLE_EQ(0.0, wrapAngle(0.0));
    EXPECT_DOUBLE_EQ(M_PI, wrapAngle(M_PI));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, wrapAngle(3.0*M_PI/2.0));
}
TEST(WrapAngleTest, Wrap) {
    EXPECT_DOUBLE_EQ(0.0, wrapAngle(2.0*M_PI));
    EXPECT_DOUBLE_EQ(M_PI/2.0, wrapAngle(5.0*M_PI/2.0));
    EXPECT_NEAR(DELTA, wrapAngle(2.0*M_PI + DELTA), EPSILON_ERROR);
}
TEST(WrapAngleTest, TwoPiWraps) {
    double input=2*M_PI;
    double expected_output=0.0;
    EXPECT_NEAR(expected_output, wrapAngle(input), EPSILON_ERROR);
}
TEST(WrapAngleTest, BoundaryWrap) {
    double input=6.283195307179586;
    double expected_output=0.0;
    EXPECT_NEAR(expected_output, wrapAngle(input), EPSILON_ERROR);
}

// --------------------- headingBetween() -------------------
TEST(HeadingBetweenTest, asin) {
    double x1=0,y1=0,x2=1,y2=0;
    EXPECT_DOUBLE_EQ(0.0, atan((y1 - y2)/(x2 - x1)));
}

const Vector2d u(0.0, 0.0);
TEST(HeadingBetweenTest, QuadrantI) {
    // Test Case for Quadrant 1 Boundaries
    Vector2d v(
        0.0,
        1.0);
    EXPECT_DOUBLE_EQ(0.0, headingBetween(u, v));

    v[0] = 1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(degToRad(45.0), headingBetween(u,v));
}

TEST(HeadingBetweenTest, QuadrantII) {
    // Test Case for Quadrant 2 Boundaries
    Vector2d v(
        1.0,
        0.0);
    EXPECT_DOUBLE_EQ(degToRad(90.0), headingBetween(u,v));

    v[0] = 1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(degToRad(135.0), headingBetween(u,v));
}
TEST(HeadingBetweenTest, QuadrantIII) {
    // Test Case for Quadrant 3 Boundaries
    Vector2d v(
        0.0,
        -1.0);
    EXPECT_DOUBLE_EQ(degToRad(180.0), headingBetween(u,v));

    v[0] = -1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(degToRad(225.0), headingBetween(u,v));
}

TEST(HeadingBetweenTest, QuadrantIV) {
    // Test Case for Quadrant 4 Boundaries
    Vector2d v(
        -1.0,
        0.0);
    EXPECT_DOUBLE_EQ(degToRad(270.0), headingBetween(u,v));

    v[0] = -1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(degToRad(315.0), headingBetween(u,v));
}

TEST(HeadingBetweenTest, Various) {
    // Some various test cases
    Vector2d u(
        1.0,
        1.0),
        v(
        1.0,
        -1.0);
    EXPECT_DOUBLE_EQ(degToRad(180), headingBetween(u,v));
    EXPECT_DOUBLE_EQ(degToRad(0), headingBetween(v,u));
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
    clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}

TEST_F(ClearEdgesTest, OneEdge) {
    EXPECT_EQ(0, GetSize());

    // Add an edge
    edge e = m_G.newEdge(GetNode(1), GetNode(2));
    EXPECT_EQ(1, GetSize());

    // Delete it
    clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}


TEST_F(ClearEdgesTest, TwoEdges) {
    EXPECT_EQ(0, GetSize());

    // Add two edges
    edge e1 = m_G.newEdge(GetNode(1), GetNode(2));
    edge e2 = m_G.newEdge(GetNode(2), GetNode(1));
    EXPECT_EQ(2, GetSize());

    // Delete them
    clearEdges(m_G);
    EXPECT_EQ(0, GetSize());
}

