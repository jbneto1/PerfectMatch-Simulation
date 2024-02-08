#include <iostream>

#include "Manager/Manager.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"

int main(int argc, char *argv[])
{
    try
    {
        Logger &logger = Logger::getInstance(spdlog::level::debug);
        logger.setPattern(std::string("[%^%l%$] %v"));       // Set logging pattern
        logger.trace("Logger instantiated and pattern set"); // Add logging
        Manager manager = Manager(logger);                   // Create manager with specified control cycle and logger
        logger.trace("Manager instantiated.");               // Add logging
        // manager.run(stringToBool(argv[1]));
        // TODO: Refactor visualizer component initialization. It should only start if run method is called.
        manager.runOfflineAnalysis("docs/logs/sensor_data_2024-02-07_09-02-06.txt");
    }
    catch (const std::exception &e)
    { // Catch any thrown exceptions
        std::cerr << "An error occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}