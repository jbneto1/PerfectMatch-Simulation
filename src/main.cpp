#include <iostream>

#include "Manager/Manager.h"
#include "config/config.h"
#include "Logger/logger.h"
#include "utils/utils.h"

int main(int argc, char *argv[])
{
    OperationalMode mode = OperationalMode::Offline;
    std::string logPath = "docs/logs/robot_move_diagonal_outliers_entrance_09_07_20_54_thesis.txt";

    try
    {
        Logger &logger = Logger::getInstance(spdlog::level::debug);
        logger.setPattern(std::string("[%^%l%$] %v"));
        logger.trace("Logger instantiated and pattern set");
        Manager manager = Manager(logger, mode);
        logger.trace("Manager instantiated.");

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
