//
// Created by jabra on 6/10/2023.
//

#ifndef AMR_PROJECT_MANAGER_H
#define AMR_PROJECT_MANAGER_H

#include <condition_variable>
#include <thread>
#include <optional>

#include "asio/signal_set.hpp"

#include "SimTwoInterface/SimTwoInterface.h"
#include "Localization/Localization.h"
#include "Logger/logger.h"
#include "AMRController/AMRController.h"
#include "Visualizer/Visualizer.h"
#include "OfflineAnalysis/OfflineAnalysis.h"

class Manager {
public:

    Manager(Logger &logger);

    ~Manager();

    void run();

    void runOfflineAnalysis(const std::string& logFilePath);

private:
    Logger& logger;
    AMRController controller;
    Localization localization;
    SimTwoInterface interface;
    Visualizer visualizer;
    std::thread visThread;
    asio::any_io_executor exec;

    asio::signal_set signals_;
    std::promise<void> CtrlCPromise;
    std::mutex PM_m;
    
    void setupSignalHandler();
    void stop();

    void onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                        AMRController &controller, Logger &logger);


    //offline analysis
    OfflineAnalysis offlineAnalysis; // OfflineAnalysis instance
    bool isOfflineMode = false; // Flag to indicate if running in offline mode
};
#endif //AMR_PROJECT_MANAGER_H
