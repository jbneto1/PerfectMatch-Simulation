// Manager.cpp

#include "Manager.h"

Manager::Manager(Logger &logger)
        : logger(logger),
          controller(logger),
          localization(logger),
          interface(logger, localization, controller),
          visualizer(localization, PM_m),
          visThread(),
          signals_(interface.getIoContext(), SIGINT),
          CtrlCPromise() {

    logger.trace("SIGINT signal handler registered with asio.");
    visThread = std::thread(&Visualizer::render, &visualizer);
    setupSignalHandler();
}

Manager::~Manager() {
    logger.trace("Manager destructor called.");
}

void Manager::stop() {
    logger.trace("Initiating stop procedure...");  // Start of stop
    signals_.cancel(); // cancel signal set

    if(visThread.joinable()) {
        visualizer.stop();  // stop the visualization thread
        visThread.join();
    }

    interface.stopIoContext();
    logger.trace("Stop procedure completed.");  // End of stop
}
void Manager::setupSignalHandler() {
    // Register the signal handler with asio
    try {

        signals_.async_wait([this](const asio::error_code &error, int signal_number) {
            logger.trace("Signal received: " + std::to_string(signal_number));
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

    interface.runIoContext();

    logger.trace("Terminating program...");
}

// This is the function that will be called when data is received.
void Manager::onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                             AMRController &controller, Logger &logger) {

    logger.trace("Data received. Handler callback called.");
    auto [encs, GT_pose, optLaserReadings] = interface.getSensorData(data);
    
    //Logging the sensors' data
    logger.fileLog_bag(encs, GT_pose, optLaserReadings); 

    if (optLaserReadings.has_value() && localization.getFirstFlag()) {
        localization.getPM().ProcessLaserPoints(optLaserReadings.value());
        localization.processData_w_PM(encs, GT_pose, optLaserReadings.value());
    } else if (optLaserReadings.has_value()) {
        localization.getPM().ProcessLaserPoints(optLaserReadings.value());
        // localization.getPM().ProcessLaserPoints(optLaserReadings.value(), localization.getPreviousPose(), GT_pose);
        // localization.setPreviousPose(GT_pose);
        localization.processData_w_PM(encs, GT_pose, optLaserReadings.value());
    } else {
        localization.processData_wo_PM(encs, GT_pose);
    }

    visualizer.update(GT_pose, localization.getPose(), optLaserReadings);
    logger.trace("Processing Perfect Match.");
}