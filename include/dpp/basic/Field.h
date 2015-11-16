#ifndef _DPP_FIELD_H_
#define _DPP_FIELD_H_

#include <cmath>
#include <limits>

#include <dpp/basic/basic.h>
#include <dpp/basic/Util.h>
#include <dpp/basic/FieldTrack.h>

namespace dpp {

#define DPP_FIELD_DEFAULT_COVERAGE_WIDTH  1

typedef ogdf::List<DPoint>         PolyVertexList;
typedef ogdf::ListIterator<DPoint> PolyVertexIterator;
typedef ogdf::ListConstIterator<DPoint> PolyVertexConstIterator;

class Field 
{
public:
    /*
    Field()
        : m_poly(),
          m_coverageWidth(DPP_FIELD_DEFAULT_COVERAGE_WIDTH)
    { }
    */

    Field(DPolygon poly, double coverageWidth = DPP_FIELD_DEFAULT_COVERAGE_WIDTH)
        : m_coverageWidth(coverageWidth),
          m_poly(poly)
    { 
        m_poly.unify(); // delete duplicates
        computeBoundingBox();
    }

    Field(PolyVertexList vertices, double coverageWidth = DPP_FIELD_DEFAULT_COVERAGE_WIDTH);

    ~Field()
    { }

    double coverageWidth(void) {
        return m_coverageWidth;
    }

    void coverageWidth(double e) {
        m_coverageWidth = e;
    }

    bool isCcw(void);

    bool isConvex(void);

    PolyVertexIterator getVertexWithMinX(void) {
        return m_minXVertex;
    }

    PolyVertexIterator getVertexWithMaxX(void) {
        return m_maxXVertex;
    }

    PolyVertexIterator getVertexWithMinY(void) {
        return m_minYVertex;
    }

    PolyVertexIterator getVertexWithMaxY(void) {
        return m_maxYVertex;
    }
/*
    DPolygon *polygon(void) {
        return &m_poly;
    }
*/
    const DPolygon *polygon(void) const {
        return const_cast<const DPolygon*>(&m_poly);
    }

    int findMinimumWidth(double &width, double &angle);

    int addNodesFromGrid(ogdf::Graph &G, ogdf::GraphAttributes &GA);
    
    int generateFieldTracks(FieldTrackList &tracks);

    friend ostream& operator<<(ostream& os, const Field& f);
        
private:
    DPolygon m_poly;
    double m_coverageWidth;
    PolyVertexIterator m_minXVertex, m_minYVertex, m_maxXVertex, m_maxYVertex;
    void computeBoundingBox(void);
   
};

inline ostream& operator<<(ostream& os, const Field& f) {
    DPP_ASSERT(f.polygon()->size() > 0);
    PolyVertexConstIterator iter;
    for ( iter = f.polygon()->begin(); iter != f.polygon()->end(); iter++ ) {
        os << "    " << *iter << std::endl;
    }
    return os;
}

/// Sweep-line used for generating field tracks. Field must be convex.
class FieldTrackSweepLine : public Line2d
{
public:
    FieldTrackSweepLine(DSegment s)
        : Line2d(s)
    { }

    bool intersectingTrack(const Field *field, FieldTrack &track);

private:
};


// Non-member functions related to Field
// FIXME move to util?
inline Vector2d segmentToVector(DSegment s) {
    return Vector2d(s.dx(), s.dy());
}

/**
 * Find the distance between the point p and the caliper line formed by
 * the caliper's unit vector and incident point.
 */
inline double distanceToCaliper(DPoint p, DPoint calPoint, Vector2d calVector) {
    DPoint calPoint2(calPoint.m_x + calVector.x(), calPoint.m_y + calVector.y());
    // perpendicular vector to caliper
    Vector2d v(calPoint2.m_y - calPoint.m_y, -(calPoint2.m_x - calPoint.m_x)); 
    v = v.normalized(); 
    // vector between p and caliper
    Vector2d r(calPoint.m_x - p.m_x, calPoint2.m_y - p.m_y);

    return fabs(v.dot(r));
}

bool findPolySegmentWithAngle(double angle, const DPolygon *poly, DSegment &seg, bool dir=true);

} // namespace dpp

#endif // _DPP_FIELD_H_
