#ifndef _DPP_FIELD_TRACK_H_
#define _DPP_FIELD_TRACK_H_

#include <dpp/basic/basic.h>
#include <dpp/basic/Util.h>

using ogdf::DPoint;
using ogdf::DPolygon;
using ogdf::DSegment;

namespace dpp {

class FieldTrack : private DLine
{
public:
    using DLine::dx;
    using DLine::dy;
    using DLine::isHorizontal;
    using DLine::isVertical;
    using DLine::length;
    using DLine::slope;
    using DLine::start;
    using DLine::end;

    FieldTrack(void)
    { }

    FieldTrack(DSegment s)
        : DLine(s.start(), s.end())
    {

    }

    FieldTrack(DPoint start, DPoint end)
        : DLine(start, end)
    {

    }
    /*
    DPoint endNode1(void) {
        return m_endNode1;
    }

    DPoint endNode2(void) {
        return m_endNode2;
    }
    */

    double angle(void) {
        DLine line(start(), end());
        return angleOfLine(line);
    }
/*
    void addEndsToGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
        // TODO move to .cpp, and add edges as well
        DPP_ASSERT(start() != end()); // FIXME add check for node NOT in graph?
        m_endNode1 = G.newNode();
        GA.x(m_endNode1) = start().m_x;
        GA.y(m_endNode1) = start().m_y;
        m_endNode2 = G.newNode();
        GA.x(m_endNode2) = end().m_x;
        GA.y(m_endNode2) = end().m_y;
    }
    */
    
private:
    //ogdf::NodeSet m_endPointNodes;
    ogdf::node m_endNode1, m_endNode2;
};

} // namespace dpp

#endif // _DPP_FIELD_TRACK_H_
