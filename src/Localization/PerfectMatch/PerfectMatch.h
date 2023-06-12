//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_PERFECTMATCH_H
#define PERFECTMATCH_SIMULATION_PERFECTMATCH_H

#include <config.h>

class PerfectMatch {
public:
    PerfectMatch();
    Pose match(const std::array<double, 720> &data);
};

#endif //PERFECTMATCH_SIMULATION_PERFECTMATCH_H
