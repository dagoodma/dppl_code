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
#include <dpp/planner/WaypointSequencePlanner.h>

using std::vector;

#define TURN_RADIUS         5.0
#define INITIAL_HEADING     1.0
#define PLANNING_ALGORITHM  dpp::DtspPlanningAlgorithm::RANDOMIZED
#define PLANNING_ALGORITHM_NAME "Randomized"


// Test initialiation is correct
TEST (WaypointSequencePlanner, ConstructsCorrectly) {
    dpp::WaypointSequencePlanner *p = new dpp::WaypointSequencePlanner();

    EXPECT_FALSE(p->haveSolution());
    EXPECT_EQ(0, p->waypointCount());
    EXPECT_FALSE(p->containsWaypoint(0,0));

    // Turn radius specifiable
    p = new dpp::WaypointSequencePlanner(TURN_RADIUS);
    EXPECT_EQ(TURN_RADIUS, p->turnRadius());

    // Initial heading specifiable
    p = new dpp::WaypointSequencePlanner(TURN_RADIUS, INITIAL_HEADING);
    EXPECT_EQ(INITIAL_HEADING, p->initialHeading());

    // Algorithm specifiable
    p = new dpp::WaypointSequencePlanner(TURN_RADIUS, INITIAL_HEADING);
    p->algorithm(PLANNING_ALGORITHM);
    EXPECT_EQ(PLANNING_ALGORITHM_NAME, p->algorithmName());
}

 // Test dies without solution
TEST(WaypointSequencePlanner, DeathWithoutSolution) {
    dpp::WaypointSequencePlanner p;
    ASSERT_EQ(0, p.waypointCount());
    ASSERT_EQ(0, p.haveSolution());

    EXPECT_DEATH(p.newWaypointSequenceId(0), "Assertion failed: \\(m_haveSolution\\).*");
}

// Test dies without waypoints
TEST(WaypointSequencePlanner, DeathWithoutWaypoints) {
    dpp::WaypointSequencePlanner p;
    vector<dpp::Waypoint> emptyList;

    EXPECT_DEATH(p.planWaypointSequence(), "Assertion failed: \\(waypointCount\\(\\) > 1\\).*");
    EXPECT_DEATH(p.addWaypoints(emptyList), "Assertion failed: \\(list.size\\(\\) > 0)");
}

// ** Data-driven tests...
const vector<dpp::Waypoint> waypointList {
    {0, 0},
    {5.1*TURN_RADIUS, 0.0},
    {5.1*TURN_RADIUS, -3.3*TURN_RADIUS}
};

vector<int> waypointListSolutions {-1, 1, 2, 3};

const vector<dpp::Waypoint> waypointList2 {
    {0, 0},
    {-6*TURN_RADIUS, -1*TURN_RADIUS},
    {-15*TURN_RADIUS, 0.0},
    {5.1*TURN_RADIUS, 0*TURN_RADIUS}
};

vector<int> waypointList2Solutions {-1, 1, 3, 2, 4};

const dpp::Waypoint newWaypoint = {-15, -15};

using ::testing::Test;
using ::testing::Values;
using ::testing::ValuesIn;

class WaypointSequencePlannerTest : public Test {
public:
    WaypointSequencePlannerTest()
    { }

    virtual ~WaypointSequencePlannerTest() { }

    virtual void SetUp() {
        m_planner.initialHeading(INITIAL_HEADING);
        m_planner.turnRadius(TURN_RADIUS);
        m_planner.addWaypoints(waypointList);
    } // SetUp()

    virtual void TearDown() {
    } // TearDown()

    bool plan(void) {
        return m_planner.planWaypointSequence();
    }

    bool verifySolution(vector<int> solution) {
        int oldIndex = 1;
        for(const auto& newIndex : solution) {
            if (newIndex < 0) continue; // skip first element

            if (newIndex != m_planner.newWaypointSequenceId(oldIndex++)) {
                return false;
            }
        }
        return true;
    }

protected:
    dpp::WaypointSequencePlanner m_planner;

}; // WaypointSequencePlannerTest

// Test for waypoints added to planner
TEST_F(WaypointSequencePlannerTest, WaypointsAdded) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());

    for (const auto& waypoint : waypointList) {
        EXPECT_TRUE(m_planner.containsWaypoint(waypoint));
    }
}

// Test for adding additional waypoints
TEST_F(WaypointSequencePlannerTest, AddMoreWaypoints) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());
    int expectedWaypointCount = waypointList.size() + 1;

    EXPECT_EQ(expectedWaypointCount, m_planner.addWaypoint(newWaypoint));
    EXPECT_EQ(expectedWaypointCount, m_planner.waypointCount());
    EXPECT_TRUE(m_planner.containsWaypoint(newWaypoint));
}

// Test for death when adding duplicate waypoints
TEST_F(WaypointSequencePlannerTest, DeathWithDuplicateWaypoints) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());

    for (const auto& waypoint : waypointList) {
        EXPECT_DEATH(m_planner.addWaypoint(waypoint),
            "Assertion failed: \\(!containsWaypoint\\(waypoint\\)\\).*");
    }
}

// Test for overwriting waypoints with list
TEST_F(WaypointSequencePlannerTest, AddNewList) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());
    int expectedWaypointCount = waypointList2.size();

    m_planner.addWaypoints(waypointList2);
    EXPECT_EQ(expectedWaypointCount, m_planner.waypointCount());
}

// Test for solution
TEST_F(WaypointSequencePlannerTest, CorrectSolution) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());
    ASSERT_FALSE(m_planner.haveSolution());

    // Solve
    EXPECT_TRUE(m_planner.planWaypointSequence());
    EXPECT_TRUE(m_planner.haveSolution());

    EXPECT_TRUE(verifySolution(waypointListSolutions));
    
}

// Test for solution to waypointList2
TEST_F(WaypointSequencePlannerTest, CorrectSolution2) {
    ENABLE_DEBUG_TEST();

    // Change the waqypoint list
    m_planner.addWaypoints(waypointList2);
    ASSERT_EQ(waypointList2.size(), m_planner.waypointCount());
    ASSERT_FALSE(m_planner.haveSolution());    

    // Solve
    EXPECT_TRUE(m_planner.planWaypointSequence());
    EXPECT_TRUE(m_planner.haveSolution());

    EXPECT_TRUE(verifySolution(waypointList2Solutions));
    DISABLE_DEBUG_TEST();
}

// Test for death with invalid old index
TEST_F(WaypointSequencePlannerTest, DeathWithBadIndex) {
    ASSERT_EQ(waypointList.size(), m_planner.waypointCount());
    ASSERT_FALSE(m_planner.haveSolution());
    ASSERT_TRUE(m_planner.planWaypointSequence());

    EXPECT_DEATH(m_planner.newWaypointSequenceId(0),
        "Assertion failed: \\(oldIndex > 0 && oldIndex <= waypointCount\\(\\)\\).*");
    EXPECT_DEATH(m_planner.newWaypointSequenceId(m_planner.waypointCount() + 1),
        "Assertion failed: \\(oldIndex > 0 && oldIndex <= waypointCount\\(\\)\\).*");
}
