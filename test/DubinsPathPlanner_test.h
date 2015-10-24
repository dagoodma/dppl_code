#ifndef _DUBINS_PATH_PLANNER_TEST_H_
#define _DUBINS_PATH_PLANNER_TEST_H_

#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>

#define ENABLE_DEBUG_TEST() do {\
    dpp::Logger *log = dpp::Logger::Instance(); \
    log->level(DPP_LOGGER_LEVEL_DEBUG); \
    log->verbose(DPP_LOGGER_VERBOSE_2); \
    } while(0)

#define DISABLE_DEBUG_TEST() do {\
    dpp::Logger *log = dpp::Logger::Instance(); \
    log->verbose(DPP_LOGGER_DEFAULT_VERBOSE); \
    log->level(DPP_LOGGER_DEFAULT_LEVEL); \
    } while(0)

// TODO add debug wrapper macro for tests? (DTEST_F, DTEST, DTEST_P)

#endif // _DUBINS_PATH_PLANNER_TEST_H_