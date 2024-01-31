//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_AMRCONTROLLER_H
#define PERFECTMATCH_SIMULATION_AMRCONTROLLER_H

#include <tuple>
#include "Logger/logger.h"
#include "config/config.h"
#include "MecanumKinematics/MecanumKinematics.h"

class AMRController {
public:
    AMRController(Logger &logger);
//    MecanumKinematics mecanum = MecanumKinematics();

    std::tuple<double, double, double, double> computeWheelSpeeds(Pose estimatedPose);

//    mecanum_wheels_speeds_t inverse_kinematics(const mecanum_speed_states_t vvnw)

private:
    Logger &logger;

};


#endif //PERFECTMATCH_SIMULATION_AMRCONTROLLER_H
