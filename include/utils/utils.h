//
// Created by jabra on 7/27/2023.
//

#ifndef PM_PROJECT_UTILS_H
#define PM_PROJECT_UTILS_H

#include <cmath>
#include <iomanip>
#include <sstream>

double radToDeg(double angle);

double diffAngle(const double ang1, const double ang2);

double normalizeAngle(double angle);

double degToRad(double angle);

std::string formatWithTwoDecimals(double value);

#endif //PM_PROJECT_UTILS_H
