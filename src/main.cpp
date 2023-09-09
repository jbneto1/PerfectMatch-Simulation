#include <iostream>
#include <Manager/Manager.h>
#include "config/config.h"
#include <Logger/logger.h>

int main() {
    try {
        Logger &logger = Logger::getInstance(spdlog::level::trace);
        logger.setPattern(std::string("[%^%l%$] %v"));  // Set logging pattern
        logger.trace("Logger instantiated and pattern set"); // Add logging
        Manager manager = Manager(logger); // Create manager with specified control cycle and logger
        logger.trace("Manager instantiated."); // Add logging
        manager.run(); // Start the manager
    } catch (const std::exception &e) { // Catch any thrown exceptions
        std::cerr << "An error occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}