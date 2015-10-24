#ifndef _DPP_LOGGER_H_
#define _DPP_LOGGER_H_
#include <stdio.h>
#include <fstream>

#include <ogdf/basic/Logger.h>

#include <dpp/basic/basic.h>

#define DPP_LOGGER_LEVEL_DEBUG  (dpp::Logger::Level::LL_DEBUG)
#define DPP_LOGGER_LEVEL_INFO  (dpp::Logger::Level::LL_INFO)
#define DPP_LOGGER_LEVEL_WARN  (dpp::Logger::Level::LL_WARN)
#define DPP_LOGGER_LEVEL_ERROR  (dpp::Logger::Level::LL_ERROR)

#define DPP_LOGGER_VERBOSE_0    (dpp::Logger::Verbosity::LV_VERBOSE_0)
#define DPP_LOGGER_VERBOSE_1    (dpp::Logger::Verbosity::LV_VERBOSE_1)
#define DPP_LOGGER_VERBOSE_2    (dpp::Logger::Verbosity::LV_VERBOSE_2)
#define DPP_LOGGER_VERBOSE_3    (dpp::Logger::Verbosity::LV_VERBOSE_3)


#define DPP_LOGGER_DEFAULT_VERBOSE  DPP_LOGGER_VERBOSE_0

// Set logger level to debug if compiled in debug mode, otherwise info
#ifdef DPP_DEBUG
#define DPP_LOGGER_DEFAULT_LEVEL    DPP_LOGGER_LEVEL_DEBUG
#else
#define DPP_LOGGER_DEFAULT_LEVEL    DPP_LOGGER_LEVEL_INFO
#endif

namespace dpp {

class Logger {
public:
    enum Level { LL_DEBUG, LL_INFO, LL_WARN, LL_ERROR };
    enum Verbosity { LV_VERBOSE_0 = 0, LV_VERBOSE_1, LV_VERBOSE_2, LV_VERBOSE_3 }; // ascending verbosity

    ~Logger();

    static Logger* Instance(void);

    static void initializeLogger(std::ostream &ostream);

    static void initializeLogger(std::string logFilename);

    static std::ostream & logOut(Logger::Level l = LL_INFO,
        Logger::Verbosity v = LV_VERBOSE_0);

    static std::ostream & logDebug(Logger::Verbosity v = LV_VERBOSE_0) {
        return logOut(Logger::Level::LL_DEBUG, v);
    }

    static std::ostream & logInfo(Logger::Verbosity v = LV_VERBOSE_0) {
        return logOut(Logger::Level::LL_INFO, v);
    }

    static std::ostream & logWarn(Logger::Verbosity v = LV_VERBOSE_0) {
        return logOut(Logger::Level::LL_WARN, v);
    }

    static std::ostream & logError(Logger::Verbosity v = LV_VERBOSE_0) {
        return logOut(Logger::Level::LL_ERROR, v);
    }

    Level level(void) {
        return m_level;
    }

    void level(Level l) {
        m_level = l;
    }

    Verbosity verbose(void) {
        return m_verbosity;
    }

    void verbose(int i) {
        DPP_ASSERT(i >= DPP_LOGGER_VERBOSE_0);
        m_verbosity = static_cast<Verbosity>(i);
    }

    bool isUsingFile(void) {
        return m_useFile;
    }

private:
    // Functions
    // TODO add dual stream (cout and file)
    Logger() 
        : m_logger(),
        m_logFile(),
        m_useFile(false),
        m_verbosity(DPP_LOGGER_DEFAULT_VERBOSE),
        m_level(DPP_LOGGER_DEFAULT_LEVEL)
    { }
    Logger(Logger const&) // copy constructor is private for singleton class
    { }
    Logger& operator=(Logger const&) // assignment is private ------"------
    { }

    // Attributes
    static Logger* m_pInstance;
    static std::ostream nullStream;
    ogdf::Logger m_logger;
    std::ofstream m_logFile;
    Level m_level;
    Verbosity m_verbosity;
    bool m_useFile;
};


} // namespace dpp

#endif // _DPP_LOGGER_H_