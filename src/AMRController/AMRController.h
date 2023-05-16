//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_AMRCONTROLLER_H
#define PERFECTMATCH_SIMULATION_AMRCONTROLLER_H

#include <tuple>

class AMRController {
public:
    AMRController();

    std::tuple<double, double, double, double> computeWheelSpeeds(double x, double y, double theta);
};


#endif //PERFECTMATCH_SIMULATION_AMRCONTROLLER_H
