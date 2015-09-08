#ifndef _DUBINS_H_
#define _DUBINS_H_

#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include "Util.h"
#include "Configuration.h"

// Prototypes
double dubinsDistanceToNode(ogdf::GraphAttributes &GA, Configuration &C, ogdf::node &node);

#endif // _DUBINS_H_
