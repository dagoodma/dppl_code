/*
 * Unit test for the dpp/basic/Logger.h singleton class.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
*/
#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>

#include <gtest/gtest.h>

#include <dpp/basic/Logger.h>


// Test for Instance()
TEST(LoggerSingletonTest, InstanceReturnsLogger) {
    dpp::Logger *log = dpp::Logger::Instance();
    SUCCEED();
    EXPECT_NE(nullptr, log);
}

// Test for Instance() as singleton
TEST(LoggerSingletonTest, InstanceIsSingleton) {
    dpp::Logger *log1 = dpp::Logger::Instance();

    dpp::Logger *log2 = dpp::Logger::Instance();

    EXPECT_EQ(log1, log2);
}

// Test for initializeLogger() with stream
TEST(LoggerIOTest, InitializeLoggerToStream) {
    std::ostringstream logstream;
    std::string msg("Testing message.");
    std::string expectedMsg(string("Log initialized.\n") + msg);

    dpp::Logger::initializeLogger(logstream);

    dpp::Logger::logOut() << msg;
    EXPECT_STREQ( expectedMsg.c_str(), logstream.str().c_str());
}


// Test for initializeLogger() with file
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push 
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(LoggerIOTest, InitializeLoggerToFile) {
    std::string msg("Testing message.");
    std::string expectedMsg(string("Log initialized.\n") + msg);
    // Create a temporary file
    string logFilename = (std::tmpnam(nullptr) + string(".log"));

    // Setup the logger with file
    dpp::Logger *log = dpp::Logger::Instance();
    dpp::Logger::initializeLogger(logFilename);
    EXPECT_EQ(true, log->isUsingFile());

    // Write and read
    log->logOut() << msg;
    log->~Logger(); // to close file handle

    std::ifstream inFile;
    inFile.open(logFilename);
    ASSERT_EQ(1, inFile.is_open());
    std::string str((std::istreambuf_iterator<char>(inFile)),
                 std::istreambuf_iterator<char>());
    inFile.close();

    // Compare
    EXPECT_STREQ( expectedMsg.c_str(), str.c_str());

    // Remove the temporary file
    ASSERT_EQ(0, std::remove(logFilename.c_str()));
}

#pragma clang diagnostic pop
#pragma GCC diagnostic pop

// Value-parameterized tests for dubinsTourCost()
//#if GTEST_HAS_PARAM_TEST
using ::testing::Test;
//using ::testing::TestWithParam;
using ::testing::Values;
using ::testing::ValuesIn;

class LoggerFeatureTest : public Test {
public:
    LoggerFeatureTest() {
        dpp::Logger::initializeLogger(m_logstream);
    }

    virtual ~LoggerFeatureTest() { }

    virtual void SetUp() {
        m_log = dpp::Logger::Instance();
        dpp::Logger::initializeLogger(m_logstream);
        m_log->level(dpp::Logger::Level::LL_INFO); // starts in DEBUG mode if DPP_DEBUG is set

    } // InitializeScenario()

    virtual void TearDown() {
    } // TearDown()

    std::string GetLastLine() {
        std::string str = m_logstream.str();
        int pos = str.find_last_of('\n') + 1;
        if (pos == std::string::npos) {
            return str;
        }
        else if (pos == str.size()) {
            pos = str.substr(0, str.size() - 1).find_last_of('\n') + 1;
        }
        return str.substr(pos);
    }

    void ClearLog() {
        m_logstream.str("");
    }

protected:
    dpp::Logger *m_log;
    std::ostringstream m_logstream;

}; // class LoggerFeatureTest 

// Test for Logger level
TEST_F(LoggerFeatureTest, LoggerLevel) {
    std::string msgInfo("Testing message info.\n");
    std::string msgDebug("Testing message debug.\n");
    std::string msgWarn("Testing message warn.\n");
    std::string msgError("Testing message error.\n");

    // ----- Info level is hides debug --------
    dpp::Logger::logInfo() << msgInfo;
    EXPECT_STREQ( msgInfo.c_str(), GetLastLine().c_str());

    // Debug doesn't show
    dpp::Logger::logDebug() << msgDebug;
    EXPECT_STRNE( msgDebug.c_str(), GetLastLine().c_str());

    // Warn shows
    dpp::Logger::logWarn() << msgWarn;
    EXPECT_STREQ( msgWarn.c_str(), GetLastLine().c_str());

    // Error shows
    dpp::Logger::logError() << msgError;
    EXPECT_STREQ( msgError.c_str(), GetLastLine().c_str());

    // ------ Debug messages print with level set --------
    m_log->level(dpp::Logger::Level::LL_DEBUG);
    ClearLog();

    dpp::Logger::logInfo() << msgInfo;
    EXPECT_STREQ( msgInfo.c_str(), GetLastLine().c_str());

    // Debug doesn't show
    dpp::Logger::logDebug() << msgDebug;
    EXPECT_STREQ( msgDebug.c_str(), GetLastLine().c_str());

    // Warn shows
    dpp::Logger::logWarn() << msgWarn;
    EXPECT_STREQ( msgWarn.c_str(), GetLastLine().c_str());

    // Error shows
    dpp::Logger::logError() << msgError;
    EXPECT_STREQ( msgError.c_str(), GetLastLine().c_str());
}

// Test for logger verbosity
TEST_F(LoggerFeatureTest, LoggerVerbosity) {
    std::string msgVerbose0("Testing message verbose0.\n");
    std::string msgVerbose1("Testing message verbose1.\n");
    std::string msgVerbose2("Testing message verbose2.\n");
    std::string msgVerbose3("Testing message verbose3.\n");

    // Verbose 0 is default, nothing else shows
    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_0) << msgVerbose0;
    EXPECT_STREQ( msgVerbose0.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_1) << msgVerbose1;
    EXPECT_STRNE( msgVerbose1.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_2) << msgVerbose2;
    EXPECT_STRNE( msgVerbose2.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_3) << msgVerbose3;
    EXPECT_STRNE( msgVerbose3.c_str(), GetLastLine().c_str());

    // Increasing verbosity shows verbose messages
    m_log->verbose(DPP_LOGGER_VERBOSE_1);
    ClearLog();
    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_0) << msgVerbose0;
    EXPECT_STREQ( msgVerbose0.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_1) << msgVerbose1;
    EXPECT_STREQ( msgVerbose1.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_2) << msgVerbose2;
    EXPECT_STRNE( msgVerbose2.c_str(), GetLastLine().c_str());
    m_log->verbose(DPP_LOGGER_VERBOSE_2);
    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_2) << msgVerbose2;
    EXPECT_STREQ( msgVerbose2.c_str(), GetLastLine().c_str());

    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_3) << msgVerbose3;
    EXPECT_STRNE( msgVerbose3.c_str(), GetLastLine().c_str());
    m_log->verbose(DPP_LOGGER_VERBOSE_3);
    dpp::Logger::logInfo(DPP_LOGGER_VERBOSE_3) << msgVerbose3;
    EXPECT_STREQ( msgVerbose3.c_str(), GetLastLine().c_str());


}