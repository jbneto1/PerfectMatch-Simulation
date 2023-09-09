//
// Created by jabra on 6/10/2023.
//

#ifndef AMR_PROJECT_MANAGER_H
#define AMR_PROJECT_MANAGER_H

#include <SimTwoInterface/SimTwoInterface.h>
#include <Localization/Localization.h>
#include <Logger/logger.h>
#include <AMRController/AMRController.h>
#include <Visualizer/Visualizer.h>
#include <thread>
#include <condition_variable>
#include <asio/signal_set.hpp>
#include <optional>

class Manager {
public:

    Manager(Logger &logger);

    ~Manager();

    void run();

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


    std::array<int, 4> encoder_readings;
    Pose GT_reading;
    std::optional<std::array<LaserPoint, 720>> laserReadings;

    void setupSignalHandler();
    void stop();

    void onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                        AMRController &controller, Logger &logger);
};
#endif //AMR_PROJECT_MANAGER_H
