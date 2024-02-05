//
// Created by jabra on 7/27/2023.
//

#include "../../include/utils/utils.h"

double radToDeg(double angle)
{
    return (angle * 180 / M_PI);
}

double diffAngle(const double ang1, const double ang2)
{
    double result = ang1 - ang2;

    if (result < -M_PI)
    {
        result += 2 * M_PI;
    }
    else if (result > M_PI)
    {
        result -= 2 * M_PI;
    }

    return result;
}

double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle <= -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double degToRad(double angle)
{
    return (angle * M_PI / 180);
}

std::string formatWithTwoDecimals(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
}
