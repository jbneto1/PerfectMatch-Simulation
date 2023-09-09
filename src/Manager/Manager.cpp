// Manager.cpp

#include "Manager.h"

Manager::Manager(Logger &logger)
        : logger(logger),
          controller(logger),
          localization(logger),
          interface(logger, localization, controller),
          visualizer(localization),
          visThread(),
          signals_(interface.getIoContext(), SIGINT),
          CtrlCPromise(),
          encoder_readings({0, 0, 0, 0}),
          GT_reading(),
          laserReadings({}) {

    logger.trace("SIGINT signal handler registered with asio.");
    visThread = std::thread(&Visualizer::render, &visualizer);
    setupSignalHandler();
}

Manager::~Manager() {
}

void Manager::stop() {
    signals_.cancel(); // cancel signal set

    if(visThread.joinable()) {
        visualizer.stop();  // stop the visualization thread
        visThread.join();
    }

    interface.stopIoContext();
}
void Manager::setupSignalHandler() {
    // Register the signal handler with asio
    try {

        signals_.async_wait([this](const asio::error_code &error, int signal_number) {
            if (!error) {
                if (signal_number == SIGINT) {
                    this->logger.trace("SIGINT caught.");
                    this->stop();
                } else {
                    this->logger.debug("Unexpected signal: " + std::to_string(signal_number));
                }
            } else {
                this->logger.error("Error in signal handler: " + error.message());
            }
        });
    } catch (const std::exception &e) {
        this->logger.error("Exception in signal handler: " + std::string(e.what()));
    }
}

void Manager::run() {

    // Register callback
    interface.registerCallback([&](const std::string &data) {
        onDataReceived(data, interface, localization, controller, logger);
    });

    logger.debug("Waiting for simulator.");

    // auto work = asio::make_work_guard(exec);
    interface.runIoContext();
    // Perform cleanup activities...

    logger.trace("Terminating program...");
}

// This is the function that will be called when data is received.
void Manager::onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                             AMRController &controller, Logger &logger) {
    logger.trace("Data received. Handler callback called.");
    auto [encoder_readings, GT_reading, optLaserReadings] = interface.getSensorData(data);

    if (optLaserReadings.has_value()) {
        laserReadings = optLaserReadings;
        localization.getPM().ProcessLaserPoints(laserReadings.value());
        localization.processData(encoder_readings, GT_reading, laserReadings.value());
    } else {
        laserReadings.reset(); // Clear the optional
        localization.processData(encoder_readings, GT_reading);
    }

    visualizer.update(GT_reading, localization.getPose(), laserReadings);
    logger.trace("Processing Perfect Match.");
}