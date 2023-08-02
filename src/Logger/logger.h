// Logger.h

#ifndef PERFECTMATCH_SIMULATION_LOGGER_H
#define PERFECTMATCH_SIMULATION_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/common.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/fmt/bin_to_hex.h> // Added to enable binary to hex conversion
#include <memory>
#include <iostream>

class Logger {
public:
    static Logger& getInstance(spdlog::level::level_enum level);

    void trace(const std::string& message);
    void debug(const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void fileLog(const std::string& message);  // Added method for file-only logging

    void set_level(const spdlog::level::level_enum log_level);
    void setPattern(const std::string &format);
    void deactivate_Loggers();

private:
    explicit Logger(spdlog::level::level_enum level);
    std::shared_ptr<spdlog::logger> logger;
    std::shared_ptr<spdlog::logger> fileLogger;  // Added file-only logger
};

#endif //PERFECTMATCH_SIMULATION_LOGGER_H
