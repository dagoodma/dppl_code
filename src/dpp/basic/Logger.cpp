/*
 * Single Logger class implementation that uses ogdf::Logger and supports a log file.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <cstddef>

#include <dpp/basic/Logger.h>

namespace dpp {

// Global static pointer used to ensure a single instance of the class.
Logger* Logger::m_pInstance = nullptr; 
std::ostream Logger::nullStream(0);

/* 
 * Initialize logging for default stream.
 */
Logger* Logger::Instance(void)
{
    // Only allow one instance of class to be generated.
    if (!m_pInstance)   {
        m_pInstance = new Logger;
    }

    return m_pInstance;
}

/*
 * Deconstructor for log file. Closes file, and destructs ogdf::Logger.
 */
Logger::~Logger() {
    if (m_useFile) {
        m_logFile.close();
    }
    m_pInstance = nullptr;
}

/* 
 * Initialize logging for writing to an output stream.
 */
void Logger::initializeLogger(std::ostream &ostream) {
    Logger *log  = Logger::Instance();
    log->m_logger.setWorldStream(ostream);
    log->m_logger.lout() << "Log intialized." << endl;
}

/* 
 * Initialize logging for writing to an output file.
 */
void Logger::initializeLogger(std::string logFilename) {
    Logger *log  = Logger::Instance();
    log->m_logFile.open(logFilename, std::fstream::out | std::fstream::trunc);
    log->m_useFile = true;
    initializeLogger(log->m_logFile);
}

std::ostream & Logger::logOut(Logger::Level l) {
    Logger *log = Logger::Instance();
    // TODO add verbose mode
#ifndef DPP_DEBUG
    if (log->m_loglevel <= l) {
        return log->m_logger.lout();
    }
#else
    return log->m_logger.lout();
#endif
    return nullStream;
}

} // namespace dpp 