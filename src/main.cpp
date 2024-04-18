#include <iostream>

#include "Manager/Manager.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"

int main(int argc, char *argv[])
{
    OperationalMode mode = OperationalMode::Offline;
    std::string logPath = "docs/logs/log_2024-04-18_11-52-00.txt";

    try
    {
        Logger &logger = Logger::getInstance(spdlog::level::trace);
        logger.setPattern(std::string("[%^%l%$] %v"));       // Set logging pattern
        logger.trace("Logger instantiated and pattern set"); // Add logging
        Manager manager = Manager(logger, mode);             // Create manager with specified control cycle and logger
        logger.trace("Manager instantiated.");               // Add logging

        if (mode == OperationalMode::Online)
        {
            logger.info("Running in online mode.");
            manager.run(stringToBool(argv[1]));
        }
        else if (mode == OperationalMode::Offline)
        {
            logger.info("Running in offline mode.");
            manager.runOfflineAnalysis(logPath);
        }
    }
    catch (const std::exception &e)
    { // Catch any thrown exceptions
        std::cerr << "An exception occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
