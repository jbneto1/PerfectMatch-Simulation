//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
#define PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H

#include <tuple>
#include <vector>

class SimTwoInterface {
public:
    SimTwoInterface();

    std::tuple<std::vector<double>, std::vector<double>> getSensorData();
    void setWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed);
};



#endif //PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
