// main.cpp

#include <iostream>
#include "Manager/Manager.h"
#include "config.h"

int main() {
    try {
        Logger &logger = Logger::getInstance(spdlog::level::info);
        logger.setPattern(std::string("[%^%l%$] %v"));  // Set logging pattern
        Manager manager = Manager(logger, CONTROL_CYCLE); // Create manager with specified control cycle and logger
        manager.run(); // Start the manager
    } catch (const std::exception &e) { // Catch any thrown exceptions
        std::cerr << "An error occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
