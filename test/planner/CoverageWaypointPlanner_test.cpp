/*
 * Unit test for dpp/planner/WaypointSequencePlanner.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include <DubinsPathPlanner_test.h>
#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/planner/CoverageWaypointPlanner.h>

using std::vector;

using dpp::VertexList;

// Parameters to use
#define TURN_RADIUS         5.0
#define SENSOR_WIDTH        2.0
#define INITIAL_HEADING     1.0
#define PLANNING_ALGORITHM  dpp::CppPlanningAlgorithm::BOUSTROPHEDON
#define PLANNING_ALGORITHM_NAME "Boustrophedon"


// Test initialiation is correct
TEST (CoverageWaypointPlannerTest, ConstructsCorrectly) {
    // Verify initial state
    {
        dpp::CoverageWaypointPlanner *p = new dpp::CoverageWaypointPlanner();

        EXPECT_FALSE(p->haveSolution());
        EXPECT_EQ(0, p->waypointCount());
    }

    // Turn radius specifiable
    {
        dpp::CoverageWaypointPlanner *p = new dpp::CoverageWaypointPlanner(TURN_RADIUS);
        EXPECT_EQ(TURN_RADIUS, p->turnRadius());
    }

    // Sensor width specifiable
    {
        dpp::CoverageWaypointPlanner *p = new dpp::CoverageWaypointPlanner(TURN_RADIUS, SENSOR_WIDTH);
        EXPECT_EQ(SENSOR_WIDTH, p->sensorWidth());
    }
    // Initial configuration specifiable
    {
        dpp::CoverageWaypointPlanner *p = new dpp::CoverageWaypointPlanner(TURN_RADIUS, SENSOR_WIDTH);
        dpp::VehicleConfiguration initialConfig(0, 0, INITIAL_HEADING);
        p->initialConfiguration(initialConfig);
        EXPECT_TRUE(initialConfig == p->initialConfiguration());
    }
    // Algorithm specifiable
    {
        dpp::CoverageWaypointPlanner *p = new dpp::CoverageWaypointPlanner(TURN_RADIUS, SENSOR_WIDTH);
        p->algorithm(PLANNING_ALGORITHM);
        EXPECT_EQ(PLANNING_ALGORITHM_NAME, p->algorithmName());
    }
}

// Test dies without solution
TEST(CoverageWaypointPlannerTest, DeathWithoutSolution) {
    dpp::CoverageWaypointPlanner p;

    ASSERT_EQ(0, p.vertexCount());
    ASSERT_FALSE(p.haveSolution());

    dpp::WaypointList list;
    EXPECT_DEATH(list = p.waypointList(), "Assertion failed: \\(haveSolution\\(\\).*");
}

// Test dies without polygon
TEST(CoverageWaypointPlannerTest, DeathWithoutPolygon) {
    dpp::CoverageWaypointPlanner p;

    ASSERT_EQ(0, p.vertexCount());
    ASSERT_FALSE(p.haveSolution());

    EXPECT_DEATH(p.planCoverageWaypoints(), "Assertion failed: \\(vertexCount\\(\\) >= 3\\).*");
}

// ------- Test data --------
// Convex polygon
const VertexList rightTriangleVertices {
    {0, 0},
    {-10.0, 10.0},
    {-10.0, 0}
};

// Non-convex polygon
const VertexList nonConvexVertices {
    {0, 0},
    {-10, 0},
    {-10.0, -10.0},
    {10, -10},
    {10, -5},
    {0, -5}
};


// Test adding vertices
TEST(CoverageWaypointPlannerTest, AddsVertices) {
    dpp::CoverageWaypointPlanner p;

    ASSERT_EQ(0, p.vertexCount());
    ASSERT_FALSE(p.haveSolution());

    int expectedN = rightTriangleVertices.size();
    EXPECT_EQ(expectedN, p.addPolygonVertices(rightTriangleVertices));
    EXPECT_EQ(expectedN, p.vertexCount());
}

// Test dies with non-convex polygon
TEST(CoverageWaypointPlannerTest, DeathWithNonConvexPolygon) {
    ENABLE_DEBUG_TEST();
    dpp::CoverageWaypointPlanner p;

    ASSERT_EQ(0, p.vertexCount());
    ASSERT_FALSE(p.haveSolution());

    int expectedN = nonConvexVertices.size();
    ASSERT_EQ(expectedN, p.addPolygonVertices(nonConvexVertices));
    ASSERT_EQ(expectedN, p.vertexCount());

    EXPECT_FALSE(p.planCoverageWaypoints());
}
