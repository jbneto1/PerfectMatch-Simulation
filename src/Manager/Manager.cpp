// Manager.cpp

#include "Manager.h"

Manager::Manager(Logger &logger)
    : logger(logger),
      controller(logger),
      localization(logger),
      interface(logger, localization, controller),
      visualizer(localization, PM_m),
      visThread(),
      signals_(interface.getIoContext()),
      CtrlCPromise()
{

    logger.trace("SIGINT signal handler registered with asio.");
    // Start the visualization thread
    setupSignalHandler();
    logger.trace("Thread T2 instantiated.");
    visThread = std::thread(&Visualizer::render, &visualizer);
    {
        std::lock_guard<std::mutex> lock(visualizer.readinessMutex);
        visualizer.isReadyForRendering = true;
    }
    visualizer.readinessCV.notify_one();
}

Manager::~Manager()
{
    logger.trace("Manager destructor called.");
    if (visThread.joinable())
    {
        visualizer.stop(); // stop the visualization thread
        visThread.join();
    }
}

void Manager::stop()
{
    logger.trace("Initiating stop procedure..."); // Start of stop
    signals_.cancel();                            // cancel signal set
    interface.stopIosContexts();
    logger.trace("Stop procedure completed."); // End of stop
}

void Manager::setupSignalHandler()
{
    signals_.add(SIGINT);
    signals_.add(SIGTERM);
    signals_.async_wait([this](const asio::error_code &error, int signal_number)
                        {
        if (!error) {
            if (signal_number == SIGINT || signal_number == SIGTERM) {
                this->logger.trace("Signal caught: " + std::to_string(signal_number));
                this->stop();
            }
        } else {
            this->logger.error("Error in signal handler: " + error.message());
            this->setupSignalHandler();
        } });
}

void Manager::run()
{

    // Register callback
    interface.registerCallback([this](const std::string &data)
                               { onDataReceived(data, interface, localization, controller, logger); });

    logger.debug("Waiting Python Script.");

    interface.runIoContextReadyMsg();

    interface.runIoContext();

    logger.trace("Terminating program...");
}

// This is the function that will be called when data is received.
void Manager::onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                             AMRController &controller, Logger &logger)
{

    logger.trace("Data received. Handler callback called.");
    auto [encs, GT_pose, optLaserReadings] = interface.getSensorData(data);
    std::string yoloData = interface.getLatestYoloData();
    // Logging the sensors' data

    logger.fileLog_bag(encs, GT_pose, optLaserReadings, yoloData);

    if (optLaserReadings.has_value() && localization.getFirstFlag())
    {
        localization.getPM().ProcessLaserPoints(optLaserReadings.value());
        localization.processData_w_PM(encs, GT_pose, optLaserReadings.value());
    }
    else if (optLaserReadings.has_value())
    {
        localization.getPM().ProcessLaserPoints(optLaserReadings.value());
        // localization.getPM().ProcessLaserPoints(optLaserReadings.value(), localization.getPreviousPose(), GT_pose);
        // localization.setPreviousPose(GT_pose);
        localization.processData_w_PM(encs, GT_pose, optLaserReadings.value());
    }
    else
    {
        localization.processData_wo_PM(encs, GT_pose);
    }

    visualizer.update(GT_pose, localization.getPose(), optLaserReadings);
    logger.trace("Processing Perfect Match.");
}