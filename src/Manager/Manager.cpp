// Manager.cpp

#include "Manager.h"

Manager::Manager(Logger &logger, OperationalMode mode)
    : logger(logger),
      mode(mode),
      controller(logger),
      localization(logger),
      localization_w_semantics(logger),
      interface(logger, localization, controller),
      visualizer(nullptr),
      visThread(),
      signals_(interface.getIoContext()),
      CtrlCPromise(),
      logData(false),
      offlineAnalysis(localization, localization_w_semantics, logger)
{
    logger.trace("SIGINT signal handler registered with asio.");
    // Start the visualization thread
    setupSignalHandler();
    if (mode == OperationalMode::Online)
    {
        visualizer = std::make_unique<Visualizer>(localization, PM_m);
        setupVisualizationThread();
    }
}

Manager::~Manager()
{
    logger.trace("Manager destructor called.");
    if (visThread.joinable())
    {
        visualizer->stop(); // stop the visualization thread
        visThread.join();
    }
}

void Manager::setupVisualizationThread()
{
    if (visualizer)
    {
        logger.trace("Thread T2 instantiated.");
        visThread = std::thread(&Visualizer::render, visualizer.get());
        {
            std::lock_guard<std::mutex> lock(visualizer->readinessMutex);
            visualizer->isReadyForRendering = true;
        }
        visualizer->readinessCV.notify_one();
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

void Manager::run(const bool logData)
{
    if (logData)
    {
        this->logData = logData;
        logger.createOnlineLoggers();
    }

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
    std::vector<BoundingBox> outliers = interface.getOutliers(yoloData);
    u_int counter = 0;
    // Logging the sensors' data

    if (logData)
        logger.fileLog_bag(encs, GT_pose, optLaserReadings, yoloData);

    localization.getPM().ProcessLaserPoints(optLaserReadings.value());
    localization.getPM().ProcessBBOutliers(optLaserReadings.value(), outliers, counter);
    localization.processData_w_PM(encs, GT_pose, optLaserReadings.value());

    visualizer->update(GT_pose, localization.getPose(), optLaserReadings);
    logger.trace("Processing Perfect Match.");
}

void Manager::runOfflineAnalysis(const std::string &logFilePath)
{
    // Create offline analysis log files.
    std::string relativePath = "../";
    relativePath.append(logFilePath);
    // Check if the file exists
    if (!std::filesystem::exists(relativePath))
    {
        logger.error("Failed opening the file. It does not exist: " + std::string(relativePath));
        return;
    }
    logger.createOfflineLoggers();

    logger.info("Processing log file at: " + std::string(relativePath));
    offlineAnalysis.processLogFile(relativePath);
}