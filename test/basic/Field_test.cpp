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
#include <dpp/basic/Util.h>
#include <dpp/basic/Field.h>

#include "DubinsPathPlanner_test.h"

using ogdf::List;
using ogdf::DPoint;
using ogdf::DSegment;
using ogdf::DPolygon;
using ogdf::ListIterator;

// ----- Test for non-member function segmentToVector  -----
const DSegment testSegment(DPoint(2,2), DPoint(6,2));
const Vector2d expectedVector(4,0);

TEST(SegmentToVectorTest, AnExample) {
	Vector2d actualVector = dpp::segmentToVector(testSegment);

	EXPECT_TRUE(expectedVector == actualVector);
}

// ---- Test for distanceToCaliper() -----
TEST(DistanceToCaliperTest, CollinearDistanceZero) {
	DPoint p(0,0);
	DPoint calP(0,0);
	Vector2d calV(0.707,0.707);

	double expectedDistance = 0;
	double actualDistance = dpp::distanceToCaliper(p, calP, calV);
}

TEST(DistanceToCaliperTest, VerticalLineDistance) {
	DPoint p(1,0);
	DPoint calP(0,0);
	Vector2d calV(0,1);

	double expectedDistance = 1;
	double actualDistance = dpp::distanceToCaliper(p, calP, calV);
}

TEST(DistanceToCaliperTest, HorizontalLine) {
	DPoint p(-0.707, -0.707);
	DPoint calP(0,0);
	Vector2d calV(1,0);

	double expectedDistance = 0.707;
	double actualDistance = dpp::distanceToCaliper(p, calP, calV);
}

TEST(DistanceToCaliperTest, AnotherLine) {
	DPoint p(0,0);
	DPoint calP(-0.707,-0.707);
	Vector2d calV(-0.707,0.707);

	double expectedDistance = 0.707;
	double actualDistance = dpp::distanceToCaliper(p, calP, calV);
}

// ------- Test data --------
// Convex polygons
const List<DPoint> rightTriangleVertices {
    {0, 0},
    {-10.0, 10.0},
    {-10.0, 0}
};

const List<DPoint> isoscelesTriangleVertices {
	{6, 2},
    {4, 4},
    {2, 2}
};

const List<DPoint> squareVertices {
	{0, 0},
    {-10, 0},
    {-10.0, -10.0},
    {0, -10}
};

const List<DPoint> rectangleVertices {
	{0, 0},
    {-10, 0},
    {-10.0, -20.0},
    {0, -20}
};

const List<DPoint> trapezoidVertices {
	{0, 0},
	{-10,10},
	{-10, -20},
	{0, -10}
};

const List<DPoint> isoscelesTrapezoidVertices {
	{0, 0},
	{10,12},
	{-10, 12},
	{-8, 0}
};

// Non-convex polygon
List<DPoint> nonConvexVertices {
	{0, 0},
    {-10, 0},
    {-10.0, -10.0},
    {10, -10},
    {10, -5},
    {0, -5}
};

// ----- Test fixture --------
/*

using ::testing::Test;
using ::testing::Values;
using ::testing::ValuesIn;

class FieldTest : public Test {
public:
    FieldTest()
    : m_field()
    { }

    virtual ~FieldTest() { }

    virtual void SetUp() {
    } // SetUp()

    virtual void TearDown() {
    } // TearDown()

    void buildField(List<DPoint> vertices) {
    	m_field = dpp::Field(vertices);
    }

protected:
	dpp::Field m_field;

}; // FieldTest
*/


// ----- Test isConvex with right triangle convex polygon ----
TEST(FieldTest, RightTriangleIsConvex) {
	dpp::Field m_field(rightTriangleVertices);

	EXPECT_TRUE(m_field.isConvex());
}

// Test isConvex with isosceles triangle convex polygon
TEST(FieldTest, IsoscelesTriangleIsConvex) {
	dpp::Field m_field(isoscelesTriangleVertices);
	EXPECT_TRUE(m_field.isConvex());
}

// Test isConvex with square convex polygon
TEST(FieldTest, SquareIsConvex) {
	dpp::Field m_field(squareVertices);
	EXPECT_TRUE(m_field.isConvex());
}

// Test isConvex with rectangular convex polygon
TEST(FieldTest, RectangleIsConvex) {
	dpp::Field m_field(rectangleVertices);
	EXPECT_TRUE(m_field.isConvex());
}

// Test isConvex with trapezoid
TEST(FieldTest, TrapezoidIsConvex) {
	dpp::Field m_field(trapezoidVertices);
	EXPECT_TRUE(m_field.isConvex());
}

// Test isConvex with trapezoid
TEST(FieldTest, IsoscelesTrapezoidIsConvex) {
	dpp::Field m_field(isoscelesTrapezoidVertices);
	EXPECT_TRUE(m_field.isConvex());
}


// Test isConvex with a non-convex polygon
TEST(FieldTest, NonConvexPolygonNotConvex) {
	dpp::Field m_field(nonConvexVertices);
	EXPECT_FALSE(m_field.isConvex());
}

// ----- Test for computeBoundingBox() -----
TEST(FieldTest, MinimumXCoordinate) {
	dpp::Field m_field(nonConvexVertices);
	DPoint expectedPoint = *(nonConvexVertices.get(1));
	DPoint actualPoint = *(m_field.getVertexWithMinX());

	EXPECT_TRUE(expectedPoint == actualPoint);
}

TEST(FieldTest, MaximumXCoordinate) {
	dpp::Field m_field(nonConvexVertices);
	DPoint expectedPoint = *(nonConvexVertices.get(3));
	DPoint actualPoint = *(m_field.getVertexWithMaxX());

	EXPECT_TRUE(expectedPoint == actualPoint);
}

TEST(FieldTest, MinimumYCoordinate) {
	dpp::Field m_field(nonConvexVertices);
	DPoint expectedPoint = *(nonConvexVertices.get(2));
	DPoint actualPoint = *(m_field.getVertexWithMinY());

	EXPECT_TRUE(expectedPoint == actualPoint);
}

TEST(FieldTest, MaximumYCoordinate) {
	dpp::Field m_field(nonConvexVertices);
	DPoint expectedPoint = *(nonConvexVertices.get(0));
	DPoint actualPoint = *(m_field.getVertexWithMaxY());

}

TEST(FieldTest, AdvanceMinYIteratorWraps) {
	dpp::Field m_field(nonConvexVertices);

	dpp::PolyVertexIterator iter = m_field.getVertexWithMinY();
	iter = m_field.polygon()->cyclicSucc(iter);
	iter = m_field.polygon()->cyclicSucc(iter);
	iter = m_field.polygon()->cyclicSucc(iter);
	iter = m_field.polygon()->cyclicSucc(iter);

	DPoint expectedPoint = *(nonConvexVertices.get(0));
	DPoint actualPoint = *iter;

	EXPECT_TRUE(expectedPoint == actualPoint);
}

// ---- Test for findMinimumWidth() on non-convex polygon fails ----
TEST(FieldTest, MinimumWidthDiesForNonConvex) {
	dpp::Field m_field(nonConvexVertices);
	double width, angle;

	EXPECT_DEATH(m_field.findMinimumWidth(width, angle),"");
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfRightTriangle) {
	//buildField(rightTriangleVertices);
	dpp::Field m_field(rightTriangleVertices);
	double expectedWidth = 7.5710678118654746;
	double expectedAngle = dpp::degToRad(135);

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfIsoscelesTriangle) {
	//buildField(isoscelesTriangleVertices);
	dpp::Field m_field(isoscelesTriangleVertices);
	double expectedWidth = 2;
	double expectedAngle = 0;

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfSquare) {
	//buildField(squareVertices);
	dpp::Field m_field(squareVertices);
	double expectedWidth = 10;
	double expectedAngle = dpp::degToRad(0);

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfRectangle) {
	dpp::Field m_field(rectangleVertices);
	double expectedWidth = 10;
	double expectedAngle = dpp::degToRad(90);

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfTrapezoid) {
	dpp::Field m_field(trapezoidVertices);
	double expectedWidth = 10;
	double expectedAngle = dpp::degToRad(90);

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// Test for findMinimumWidth() on convex polygons work
TEST(FieldTest, MinimumWidthOfIsoscelesTrapezoid) {
	dpp::Field m_field(isoscelesTrapezoidVertices);
	double expectedWidth = 12;
	double expectedAngle = 0;

	double actualWidth, actualAngle;

	EXPECT_EQ(0, m_field.findMinimumWidth(actualWidth, actualAngle));
	EXPECT_DOUBLE_EQ(expectedWidth, actualWidth);
	EXPECT_DOUBLE_EQ(expectedAngle, actualAngle);
}

// ---- Test for addNodesFromGrid() ----
// TODO write tests for this
