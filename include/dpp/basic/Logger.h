#ifndef _DPP_LOGGER_H_
#define _DPP_LOGGER_H_
#include <stdio.h>
#include <fstream>

#include <ogdf/basic/Logger.h>

#include <dpp/basic/basic.h>

namespace dpp {

class Logger {
public:
    enum Level { LL_DEBUG, LL_INFO, LL_WARN, LL_ERROR };

    ~Logger();

    static Logger* Instance(void);

    static void initializeLogger(std::ostream &ostream);

    static void initializeLogger(std::string logFilename);
    static std::ostream & logOut(Logger::Level l=
#ifndef DPP_DEBUG
        LL_INFO);
#else
        LL_DEBUG);
#endif

    static std::ostream & logDebug() {
        return logOut(Logger::Level::LL_DEBUG);
    }

    static std::ostream & logInfo() {
        return logOut();
    }

    static std::ostream & logWarn() {
        return logOut(Logger::Level::LL_WARN);
    }

    static std::ostream & logError() {
        return logOut(Logger::Level::LL_ERROR);
    }

    Level level(void) {
        return m_loglevel;
    }

    void level(Level l) {
        m_loglevel = l;
    }

private:
    // Functions
    Logger() 
        : m_logger(),
        m_logFile(),
        m_useFile(false)
    { }
    Logger(Logger const&) // copy constructor is private
    { }
    Logger& operator=(Logger const&) // assignment is private
    { }

    // Attributes
    static Logger* m_pInstance;
    static std::ostream nullStream;
    ogdf::Logger m_logger;
    std::ofstream m_logFile;
    Level m_loglevel;
    bool m_useFile;
};


} // namespace dpp

#endif // _DPP_LOGGER_H_