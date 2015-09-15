#include "gtest/gtest.h"

#include "Dubins.h"

#define TURN_RADIUS     10.02
#define EPSILON_ERROR   0.05 // error margin in length calculation

// Test for dubinsPathLength()
TEST(DubinsPathLengthTest, Zero) {
    Configuration c1(0.0,0.0,0.0), c2(0.0,0.0,0.0);
    EXPECT_EQ(0.0, dubinsPathLength(c1, c2, TURN_RADIUS));
}
