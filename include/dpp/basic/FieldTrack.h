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
    using DLine::contains;

    FieldTrack(void)
        : m_reverse(false)
    { }

    FieldTrack(DSegment s)
        : DLine(s.start(), s.end())
    { }

    FieldTrack(DPoint start, DPoint end)
        : DLine(start, end)
    { }

    ~FieldTrack(void)
    { }

    const DPoint& start(void) const {
        if (!m_reverse) {
            return DLine::start();
        }
        else {
            return DLine::end();
        }
    }

    const DPoint& end(void) const {
        if (!m_reverse) {
            return DLine::end();
        }
        else {
            return DLine::start();
        }
    }

    // Equality only compares to see if endpoints are identical. start/end order donesn't matter
    bool operator== (const FieldTrack &track) const {
        return (start() == track.start() && end() == track.end())
            || (end() == track.start() && start() == track.end());
    }

    // Not equals
    bool operator!= (const FieldTrack &track) const {
        return !(*this == track);
    }

    /*
    DPoint endNode1(void) {
        return m_endNode1;
    }

    DPoint endNode2(void) {
        return m_endNode2;
    }
    */

    /**
     * Returns the angle of the track line from start to end between [0, 2*pi).
     */
    double angle(void) const {
        DLine line(start(), end());
        if (m_reverse) {
            line = DLine(end(), start());
        }
        return angleOfLine(line);
    }

    /**
     * Returns the angle of the track line from start to end between [0, 2*pi).
     */
    double angle(bool reverse) const {
        DLine line(start(), end());
        if (reverse) {
            line = DLine(end(), start());
        }
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
    bool reverse(void) const {
        return m_reverse;
    }

    void reverse(bool rev) {
        m_reverse = rev;
    }

    friend ostream& operator<<(ostream& os, const FieldTrack& t);
    
private:
    //ogdf::NodeSet m_endPointNodes;
    //ogdf::node m_endNode1, m_endNode2;
    bool m_reverse;
};

inline ostream& operator<<(ostream& os, const FieldTrack& t) {
    return os << t.start() << ((!t.reverse())? "->" : "<-") << t.end();
}

class FieldTrackList : public ogdf::List<FieldTrack>
{
public:
    // inherit constructors
    using List<FieldTrack>::List;

    int addNodesToGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA, 
        double distance, ogdf::List<ogdf::node> &Tour,
        ogdf::NodeArray<double> &Headings) const;
    
private:
}; 

typedef ogdf::ListIterator<FieldTrack> FieldTrackListIterator;
typedef ogdf::ListConstIterator<FieldTrack> FieldTrackListConstIterator;

} // namespace dpp

#endif // _DPP_FIELD_TRACK_H_
