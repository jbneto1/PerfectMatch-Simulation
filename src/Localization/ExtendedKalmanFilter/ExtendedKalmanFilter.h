//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
#define PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H

#include <vector>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    // An example method. You can replace this with the actual methods you need
    std::vector<double> filter(const std::vector<double>& data);
};

#endif //PERFECTMATCH_SIMULATION_EXTENDEDKALMANFILTER_H
