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
#include <dpp/basic/FieldTrack.h>

#include "DubinsPathPlanner_test.h"

using ogdf::DIsEqual;
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
// FIXME remote this, not needed
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

	dpp::PolyVertexConstIterator iter = m_field.getVertexWithMinY();
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

	EXPECT_DEATH(m_field.findMinimumWidth(width, angle),"isConvex\\(\\)");
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


// ----- Test for findPolySegmentWithAngle() ------
TEST(FindPolySegmentWithAngleTest, AngleOutOfRangeDies) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();
	DSegment s;
	{
		double angle = 2*M_PI;
		EXPECT_DEATH(dpp::findPolySegmentWithAngle(angle, poly, s),
		 "0 <= angle && angle < 2\\*3\\.14159265358979323846264338327950288");
	}
	{
		double angle = -M_PI/2;
		EXPECT_DEATH(dpp::findPolySegmentWithAngle(angle, poly, s),
		 "0 <= angle && angle < 2\\*3\\.14159265358979323846264338327950288");
	}
}

TEST(FindPolySegmentWithAngleTest, EmptyPolygonDies) {
	DPolygon poly;
	DSegment s;
	double angle = 0;
	EXPECT_DEATH(dpp::findPolySegmentWithAngle(angle, &poly, s),
		"poly->size\\(\\) > 0");
}

TEST(FindPolySegmentWithAngleTest, NoEdgeWithAngle) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();
	DSegment s;
	double angle = M_PI;

	EXPECT_FALSE(dpp::findPolySegmentWithAngle(angle, poly, s));
}

TEST(FindPolySegmentWithAngleTest, OneEdgeWithAngle) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();
	DSegment s;
	double angle = 0;

	DSegment expectedSegment(DPoint(-10,0), DPoint(0,0));
	EXPECT_TRUE(dpp::findPolySegmentWithAngle(angle, poly, s));
	EXPECT_TRUE(expectedSegment == s);
}

TEST(FindPolySegmentWithAngleTest, NoEdgeWithDirection) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();
	DSegment s;
	double angle = dpp::degToRad(90);

	EXPECT_FALSE(dpp::findPolySegmentWithAngle(angle, poly, s));
}

TEST(FindPolySegmentWithAngleTest, HasEdgeWithoutDirection) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();
	DSegment s;
	double angle = dpp::degToRad(90);
	DSegment expectedSegment(DPoint(-10,10), DPoint(-10,0));

	EXPECT_TRUE(dpp::findPolySegmentWithAngle(angle, poly, s, false));
	EXPECT_TRUE(expectedSegment == s);
}


// ----- Test for FieldTrackSweepLine::intersectingTrack() ------
TEST(FieldTrackSweepLineTest, Constructs) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();

	DSegment s = poly->segment(poly->begin());
	dpp::FieldTrackSweepLine sweepLine(s);

	SUCCEED();
}

TEST(FieldTrackSweepLineTest, NoIntersectionOnEdge) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();

	DSegment s = poly->segment(poly->begin());
	dpp::FieldTrackSweepLine sweepLine(s);
	dpp::FieldTrack t;

	EXPECT_FALSE(sweepLine.intersectingTrack(&f, t));
}

TEST(FieldTrackSweepLineTest, HasIntersection) {
	dpp::Field f(rightTriangleVertices);
	const DPolygon *poly = f.polygon();

	DSegment s = poly->segment(poly->begin());
	dpp::FieldTrackSweepLine sweepLine(s);
	double sweepAngle = dpp::wrapAngle(sweepLine.angle() + M_PI/2);
	sweepLine.translatePolar(f.coverageWidth()/2, sweepAngle);
	dpp::FieldTrack actualT;
	dpp::FieldTrack expectedT(DPoint(-10,9.292893219), DPoint(-0.7071067812,0));

	EXPECT_TRUE(sweepLine.intersectingTrack(&f, actualT));
	EXPECT_TRUE(expectedT == actualT);
	/*
	EXPECT_TRUE(expectedT.start() == actualT.start());
	EXPECT_TRUE(expectedT.end() == actualT.end());
	*/
}

// ----- Test for generateFieldTracks() ------


TEST(FieldGenerateTracksTest, RightTriangle) {
	dpp::Field f(rightTriangleVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(-10,8.585786438), DPoint(-1.414213562,0)),
		dpp::FieldTrack(DPoint(-10,5.75735931), DPoint(-4.24264069,0)),
		dpp::FieldTrack(DPoint(-10,2.92893219), DPoint(-7.07106781,0)),
		dpp::FieldTrack(DPoint(-10,0.100505063), DPoint(-9.89949494,0))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}

TEST(FieldGenerateTracksTest, IsoscelesTriangle) {
	dpp::Field f(isoscelesTriangleVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(3,3), DPoint(5,3))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}

TEST(FieldGenerateTracksTest, Square) {
	dpp::Field f(squareVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(-10,-1), DPoint(0,-1)),
		dpp::FieldTrack(DPoint(-10,-3), DPoint(0,-3)),
		dpp::FieldTrack(DPoint(-10,-5), DPoint(0,-5)),
		dpp::FieldTrack(DPoint(-10,-7), DPoint(0,-7)),
		dpp::FieldTrack(DPoint(-10,-9), DPoint(0,-9))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}

TEST(FieldGenerateTracksTest, Rectangle) {
	dpp::Field f(rectangleVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(-9,0), DPoint(-9,-20)),
		dpp::FieldTrack(DPoint(-7,0), DPoint(-7,-20)),
		dpp::FieldTrack(DPoint(-5,0), DPoint(-5,-20)),
		dpp::FieldTrack(DPoint(-3,0), DPoint(-3,-20)),
		dpp::FieldTrack(DPoint(-1,0), DPoint(-1,-20))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}

TEST(FieldGenerateTracksTest, Trapezoid) {
	dpp::Field f(trapezoidVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(-9,9), DPoint(-9,-19)),
		dpp::FieldTrack(DPoint(-7,7), DPoint(-7,-17)),
		dpp::FieldTrack(DPoint(-5,5), DPoint(-5,-15)),
		dpp::FieldTrack(DPoint(-3,3), DPoint(-3,-13)),
		dpp::FieldTrack(DPoint(-1,1), DPoint(-1,-11))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}

TEST(FieldGenerateTracksTest, IsoscelesTrapezoid) {
	//ENABLE_DEBUG_TEST();
	//std::cout << std::setprecision(10);
	dpp::Field f(isoscelesTrapezoidVertices, 2);

	dpp::FieldTrackList expectedTracks({
		dpp::FieldTrack(DPoint(-9.833333333,11), DPoint(9.166666666,11)),
		dpp::FieldTrack(DPoint(-9.5,9), DPoint(7.5,9)),
		dpp::FieldTrack(DPoint(-9.166666666,7), DPoint(5.833333333,7)),
		dpp::FieldTrack(DPoint(-8.833333333,5), DPoint(4.16666666,5)),
		dpp::FieldTrack(DPoint(-8.5,3), DPoint(2.5,3)),
		dpp::FieldTrack(DPoint(-8.166666666,1), DPoint(0.8333333,1))
	});
	dpp::FieldTrackList actualTracks;

	EXPECT_GT(f.generateFieldTracks(actualTracks), 0);
	EXPECT_TRUE(expectedTracks == actualTracks);
}
