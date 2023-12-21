// Logger.cpp
#include "logger.h"


Logger::Logger(spdlog::level::level_enum level) {
    std::vector<spdlog::sink_ptr> sinks;
    try {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));

        // Initialize file-only logger
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logfile.txt", true);
        fileLogger = std::make_shared<spdlog::logger>("FileLogger", file_sink);

        spdlog::register_logger(logger);
        spdlog::register_logger(fileLogger);
        this->set_level(level);
        fileLogger->set_level(spdlog::level::off);
        fileLogger->set_pattern(std::string("%v"));
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log initialization failed: " << ex.what() << std::endl;
    } catch (const std::exception& ex) {
        std::cout << "General exception: " << ex.what() << std::endl;
    }
}


Logger& Logger::getInstance(spdlog::level::level_enum level) {
    static Logger instance = Logger(level);
    return instance;
}

void Logger::trace(const std::string &message) {
    try {
        logger->trace(message);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::debug(const std::string& message) {
    try {
        logger->debug(message);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::info(const std::string& message) {
    try {
        logger->info(message);
    } catch (const spdlog::spdlog_ex& ex) {
        
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::warn(const std::string& message) {
    try {
        logger->warn(message);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::error(const std::string& message) {
    try {
        logger->error(message);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::set_level(const spdlog::level::level_enum log_level) {
    logger->set_level(log_level);
}

void Logger::setPattern(const std::string &format) {
    logger->set_pattern(format);
}

void Logger::fileLog(const std::string& message) {
    try {
        fileLogger->info(message);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::deactivate_Loggers() {

    logger->set_level(spdlog::level::off);
    fileLogger->set_level(spdlog::level::off);

}


