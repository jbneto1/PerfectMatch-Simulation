//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
#define PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H

#include <vector>
#include "config/config.h"

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    void predict();
    void update(const Pose measurement);

    Pose getPose();
    void setPose(const Pose startPose);

private:
    Pose pose;
};

#endif //PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
