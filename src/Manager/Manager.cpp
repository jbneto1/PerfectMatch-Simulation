// Manager.cpp

#include "Manager.h"

std::atomic<bool> Manager::run_loop; // Control variable for the main run loop

Manager::Manager(Logger &logger, const double control_cycle) : logger(logger), dt(control_cycle) {
    run_loop = true;  // Initialize loop control variable
    std::signal(SIGINT, Manager::signalHandler);  // Register SIGINT handler
    logger.trace("SIGINT signal handler registered.");
    logger.trace("Manager instantiated.");
}

Manager::~Manager() {
    logger.trace("Manager destructor called.");
    // Placeholder for any cleanup tasks
}

void Manager::run() {
    // Register callback
    interface.registerCallback([&](const std::string &data) {
        onDataReceived(data, interface, localization, controller, logger);
    });

    logger.trace("Callback registered.");
    logger.trace("Program executing...");

    logger.fileLog("[GT.x],[GT.y],[GT.theta],[Match.x],[Match.y],[Match.theta],[runtime]");
    while (run_loop);  // Main run loop

    logger.trace("Terminating program...");
}

void Manager::signalHandler(int sig) {
    if (sig == SIGINT) run_loop = false;  // On SIGINT, break the main run loop
    Logger::getInstance(spdlog::level::info).trace("SIGINT caught.");
}

// This is the function that will be called when data is received.
void Manager::onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                             AMRController &controller, Logger &logger) {
    logger.trace("Data received. Handler callback called.");
    auto [encoders, groundTruth, lidarData] =  interface.getSensorData(data);
    logger.trace("Processing Perfect Match.");
    localization.processData(encoders, groundTruth, lidarData);
}
