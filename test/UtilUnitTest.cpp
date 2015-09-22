/*
 * Copyright (C) 2014-2015 DubinsAreaCoverage.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the COPYRIGHT file distributed with DubinsAreaCoverage.
*/
#include <math.h>

#include "gtest/gtest.h"
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Vector2d;

#include "Util.h"

#define EPSILON         0.01 
#define EPSILON_ERROR   0.0001  // Close enough for government work

// Test for headingToAngle()
TEST(HeadingToAngleTest, Quadrants) {
    EXPECT_DOUBLE_EQ(M_PI/2.0, headingToAngle(0.0));
    EXPECT_DOUBLE_EQ(0.0, headingToAngle(M_PI/2.0));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, headingToAngle(M_PI));
    EXPECT_DOUBLE_EQ(M_PI, headingToAngle(3.0*M_PI/2.0));
}

// Test for wrapAngle()
TEST(WrapAngleTest, NoWrap) {
    EXPECT_DOUBLE_EQ(0.0, wrapAngle(0.0));
    EXPECT_DOUBLE_EQ(M_PI, wrapAngle(M_PI));
    EXPECT_DOUBLE_EQ(3.0*M_PI/2.0, wrapAngle(3.0*M_PI/2.0));
}
TEST(WrapAngleTest, Wrap) {
    EXPECT_DOUBLE_EQ(0.0, wrapAngle(2.0*M_PI));
    EXPECT_DOUBLE_EQ(M_PI/2.0, wrapAngle(5.0*M_PI/2.0));
    EXPECT_NEAR(EPSILON, wrapAngle(2.0*M_PI + EPSILON), EPSILON_ERROR);
}

// Test for headingBetween()
TEST(HeadingBetweenTest, asin) {
    double x1=0,y1=0,x2=1,y2=0;
    EXPECT_DOUBLE_EQ(0.0, atan((y1 - y2)/(x2 - x1)));
}

const Vector2d u(0.0, 0.0);

TEST(HeadingBetweenTest, QuadrantI) {
    // Case Ia
    Vector2d v(
        0.0,
        1.0);
    EXPECT_DOUBLE_EQ(0.0, headingBetween(u, v));

    v[0] = 1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(degToRad(45.0), headingBetween(u,v));
}
TEST(HeadingBetweenTest, QuadrantII) {
    Vector2d v(
        1.0,
        0.0);
    EXPECT_DOUBLE_EQ(degToRad(90.0), headingBetween(u,v));

    v[0] = 1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(degToRad(135.0), headingBetween(u,v));
}
TEST(HeadingBetweenTest, QuadrantIII) {
    Vector2d v(
        0.0,
        -1.0);
    EXPECT_DOUBLE_EQ(degToRad(180.0), headingBetween(u,v));

    v[0] = -1.0;
    v[1] = -1.0;
    EXPECT_DOUBLE_EQ(degToRad(225.0), headingBetween(u,v));
}

TEST(HeadingBetweenTest, QuadrantIV) {
    Vector2d v(
        -1.0,
        0.0);
    EXPECT_DOUBLE_EQ(degToRad(270.0), headingBetween(u,v));

    v[0] = -1.0;
    v[1] = 1.0;
    EXPECT_DOUBLE_EQ(degToRad(315.0), headingBetween(u,v));
}

// TODO add tests for wrap around checking
TEST(HeadingBetweenTest, Various) {
    Vector2d u(
        1.0,
        1.0),
        v(
        1.0,
        -1.0);
    EXPECT_DOUBLE_EQ(degToRad(180), headingBetween(u,v));
    EXPECT_DOUBLE_EQ(degToRad(0), headingBetween(v,u));
}

// TODO parameterize these tests


