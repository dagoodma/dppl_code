/*
 * Implementation of dpp::FieldTracksCPP class for solving CPP problems
 * by decomposing the field into convex polygons, adding minimum altitude
 * tracks for each polygon, and solving for the optimal visiting order
 * with LKH by converting this GTSP into an ATSP.
 *
 * Derived from the Xin Yu's dissertation, 2015.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <dpp/basic/Logger.h>

#include <dpp/planalg/FieldTracksCpp.h>

namespace dpp {



int FieldTracksCpp::run(void) {
    Logger::logWarn() << "Field Tracks algorithm not implemented!" << std::endl;

    return SUCCESS;
}

} // namespace dpp