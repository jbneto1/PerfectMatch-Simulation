//
// Created by jabra on 6/10/2023.
//

#ifndef AMR_PROJECT_MANAGER_H
#define AMR_PROJECT_MANAGER_H

#include "SimTwoInterface/SimTwoInterface.h"
#include "Localization/Localization.h"
#include "Logger/logger.h"
#include "AMRController/AMRController.h"
#include <csignal>

class Manager {
public:

    Manager(Logger &logger, const double control_cycle);
    ~Manager();

    void run();

private:
    const double dt;
    Logger &logger;
    AMRController controller = AMRController(logger);
    Localization localization = Localization(logger, controller, dt, MAX_ITERS);
    SimTwoInterface interface = SimTwoInterface(logger, localization, controller);


    static std::atomic<bool> run_loop;

    static void signalHandler(int sig);

    void onDataReceived(const std::string &data, SimTwoInterface &interface, Localization &localization,
                       AMRController &controller, Logger &logger);
};

#endif //AMR_PROJECT_MANAGER_H
