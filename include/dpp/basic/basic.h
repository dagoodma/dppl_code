#ifndef _DPP_BASIC_H_
#define _DPP_BASIC_H_
#include <ogdf/basic/GraphAttributes.h>

#ifdef DPP_DEBUG
#include <assert.h>
#define DPP_ASSERT(expr) assert(expr);
//#define OGDF_ASSERT_IF(minLevel,expr) \
//    if (int(ogdf::debugLevel) >= int(minLevel)) { assert(expr); } else { }
//#define DPP_SET_DEBUG_LEVEL(level) ogdf::debugLevel = level;
#else
#define DPP_ASSERT(expr)
//#define DPP_ASSERT_IF(minLevel,expr)
//#define DPP_SET_DEBUG_LEVEL(level)
#endif

#define FAILURE (-1)
#define SUCCESS (0)

#define DPP_GRAPH_ATTRIBUTES ( \
            GraphAttributes::nodeGraphics | \
            GraphAttributes::edgeGraphics | \
            GraphAttributes::nodeLabel | \
            GraphAttributes::edgeStyle | \
            GraphAttributes::edgeDoubleWeight | \
            GraphAttributes::nodeStyle | \
            GraphAttributes::nodeTemplate | \
            GraphAttributes::nodeId)

namespace dpp {

} // namespace dpp

#endif // _DPP_BASIC_H_