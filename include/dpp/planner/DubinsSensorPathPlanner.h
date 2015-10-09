#ifndef _DPP_DUBINSSENSORPATHPLANNER_H_
#define _DPP_DUBINSSENSORPATHPLANNER_H_

namespace DPP {

class DubinsSensorPathPlanner : public DubinsVehiclePathPlanner {

    void polygon(DPolygon polygon);
    void polygon(List<DPoint> points);
    DPolygon polygon(void) {}
};

} // namespace DPP

#endif // _DPP_DUBINSSENSORPATHPLANNER_H_