/*
 * Copyright (C) 2014-2015 DubinsAreaCoverage.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the COPYRIGHT file distributed with DubinsAreaCoverage.
*/
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include "DubinsCurves.h"

#include "Configuration.h"
#include "Dubins.h"
#include "Util.h"

using namespace DubinsCurves;

// Have to import specifially, since Configuration clashes
using ogdf::node;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::DPoint;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::ListConstIterator;
using ogdf::NodeArray;

using std::vector;

#define TURN_RADIUS     1.0
#define EPSILON_ERROR   0.05 // error margin in length calculation

// Test for dubinsPathLength()
TEST(DubinsPathLengthTest, MinDistance) {
    Configuration c1(0.0,0.0,0.0), c2(0.0,0.0,0.0);
    EXPECT_EQ(-1, dubinsPathLength(c1, c2, TURN_RADIUS));
}

// Test against DubinsCurves lib
TEST(DubinsPathLengthTest, RSR) {
    Configuration c1(0.0,0.0,0.0),
        c2(5.0*TURN_RADIUS,0, degToRad(180.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = headingToAngle(q0[2]);
    q1[2] = headingToAngle(q1[2]);
    DubinsPath path;
    dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, RSL) {
    Configuration c1(0.0,0.0,degToRad(37)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, degToRad(340.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = headingToAngle(q0[2]);
    q1[2] = headingToAngle(q1[2]);
    DubinsPath path;
    dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, LSR) {
    Configuration c1(0.0,0.0,degToRad(245)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, degToRad(165.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = headingToAngle(q0[2]);
    q1[2] = headingToAngle(q1[2]);
    DubinsPath path;
    dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dubinsPathLength(c1, c2, TURN_RADIUS));
}

TEST(DubinsPathLengthTest, LSL) {
    Configuration c1(0.0,0.0,degToRad(170)),
        c2(5.1*TURN_RADIUS,-1.3*TURN_RADIUS, degToRad(315.0));

    double *q0, *q1;
    c1.asArray(&q0);
    c2.asArray(&q1);
    q0[2] = headingToAngle(q0[2]);
    q1[2] = headingToAngle(q1[2]);
    DubinsPath path;
    dubins_init( q0, q1, TURN_RADIUS, &path);
    double expectedLength = dubins_path_length(&path);

    EXPECT_DOUBLE_EQ(expectedLength, dubinsPathLength(c1, c2, TURN_RADIUS));
}

// Value-parameterized tests for dubinsTourCost()
//#if GTEST_HAS_PARAM_TEST
using ::testing::Test;
//using ::testing::TestWithParam;
using ::testing::Values;
using ::testing::ValuesIn;

// Some node configurations to use
const List<Configuration> nodeConfigs { 
    {0.0, 0.0, 0.0}, 
    {5.1*TURN_RADIUS, 0.0, degToRad(180)}, 
    {5.1*TURN_RADIUS, -3.3*TURN_RADIUS, degToRad(270)}
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
            GraphAttributes::nodeStyle |
            GraphAttributes::nodeTemplate |
            GraphAttributes::nodeId),
          m_nodeHeadings(m_G) {}

    virtual ~DubinsTourTest() { }

    //void InitializeScenario(List<Configuration> configs) {
    virtual void SetUp() {
        m_n = nodeConfigs.size();
        int i = 1; // node index
        //m_nodeList = new node*[m_n];
        m_nodeList.resize(m_n);

        // Build graph
        ListConstIterator<Configuration> iter;
        
        for ( iter = nodeConfigs.begin(); iter != nodeConfigs.end(); iter++ ) {
            Configuration Cu = *iter;
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
    List<node> tour;

    EXPECT_EQ(0.0, dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, false));
}

TEST_F(DubinsTourTest, ShortTourHasZeroCost) {
    ASSERT_GT(GetSize(), 0);
    List<node> tour;
    tour.pushBack(GetNode(1));

    EXPECT_EQ(0.0, dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, false));
    EXPECT_EQ(0.0, dubinsTourCost(m_G, m_GA, tour, m_nodeHeadings, TURN_RADIUS, true));
}

TEST_F(DubinsTourTest, TourReturnsCorrectCost) {
    //InitializeScenario(nodeConfigs);
    ASSERT_EQ(3, GetSize());

    List<node> tour;
    tour.pushBack(GetNode(1));
    tour.pushBack(GetNode(2));

    EXPECT_DOUBLE_EQ(6.241592653589793, dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));

    tour.pushBack(GetNode(3));
    EXPECT_DOUBLE_EQ(10.351530620781304, dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));

    EXPECT_DOUBLE_EQ(16.623390656993465, dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, true));

    tour.pushBack(GetNode(1));
    EXPECT_DOUBLE_EQ(16.623390656993465, dubinsTourCost(m_G, m_GA, tour,
        m_nodeHeadings, TURN_RADIUS, false));
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
