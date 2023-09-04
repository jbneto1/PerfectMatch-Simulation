// Manager.cpp

#include "Manager.h"

std::atomic<bool> Manager::run_loop; // Control variable for the main run loop

Manager::Manager(Logger &logger) : logger(logger),
                                                               visualizer(localization) {
    run_loop = true;  // Initialize loop control variable
    std::signal(SIGINT, Manager::signalHandler);  // Register SIGINT handler
    logger.trace("SIGINT signal handler registered.");
    visThread = std::thread(&Visualizer::render, &visualizer);
}

Manager::~Manager() {
    logger.trace("Manager destructor called.");
    if (visThread.joinable())
        visThread.join();
    // Placeholder for any cleanup tasks
}

void Manager::run() {
    // Register callback
    interface.registerCallback([&](const std::string &data) {
        onDataReceived(data, interface, localization, controller, logger);
    });

    logger.debug("Waiting for simulator.");

    while (run_loop) {
        runOptimization();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    };  // Main run loop

    logger.trace("Terminating program...");
}

void Manager::signalHandler(int sig) {
    if (sig == SIGINT) run_loop = false;  // On SIGINT, break the main run loop
    Logger::getInstance(spdlog::level::info).trace("SIGINT caught.");
}

// This is the function that will be called when data is received.
void Manager::onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                             AMRController &controller, Logger &logger) {
    std::lock_guard<std::mutex> lock(dataMutex);
    logger.trace("Data received. Handler callback called.");
    std::tie(encoder_readings, GT_reading, laserReadings) = interface.getSensorData(data);
    localization.getPM().ProcessLaserPoints(laserReadings);
    logger.trace("Processing Perfect Match.");
}

void Manager::runOptimization() {
    std::lock_guard<std::mutex> lock(dataMutex);
    localization.processData(encoder_readings, GT_reading, laserReadings);
    visualizer.update(localization.getGTPose(), localization.getPose(), laserReadings);
}
