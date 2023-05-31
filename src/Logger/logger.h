// Logger.h
#ifndef PERFECTMATCH_SIMULATION_LOGGER_H
#define PERFECTMATCH_SIMULATION_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

class Logger {
public:
    // This is a singleton class
    static Logger& getInstance();

    // Log message at various levels
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void debug(const std::string& message);

private:
    // private constructor for singleton
    Logger();

    std::shared_ptr<spdlog::logger> console;
};

#endif //PERFECTMATCH_SIMULATION_LOGGER_H
