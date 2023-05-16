//
// Created by jabra on 5/16/2023.
//

#ifndef PERFECTMATCH_SIMULATION_LOCALIZATION_H
#define PERFECTMATCH_SIMULATION_LOCALIZATION_H

#include <tuple>
#include <vector>

class Localization {
public:
    Localization();

    std::tuple<double, double, double> processData(const std::vector<double>& lidarData, const std::vector<double>& encoderData);
};


#endif //PERFECTMATCH_SIMULATION_LOCALIZATION_H
