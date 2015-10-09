/*
 * Path planner for a Dubins vehicle with a sensor. Considers the 2D turning radius
 * of the vehicle and the coverage width of the sensor.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include "Path.h"
#include "Util.h"
#include "VehicleConfiguration.h"
#include "DubinsVehiclePathPlanner.h"

namespace DPP {

class DubinsSensorPathPlanner : public PathPlanner {

    void polygon(DPolygon polygon);
    void polygon(List<DPoint> points);
    DPolygon polygon(void) {}

};

} // namespace DPP 