//
// Created by jabra on 6/5/2023.
//

#ifndef AMR_PROJECT_CONFIG_H
#define AMR_PROJECT_CONFIG_H

#include <string_view>

constexpr int MAX_BUFFER_SIZE = 16256;
constexpr int SIMTWO_RECEIVE_PORT = 9000;
constexpr int SYNCMSG_RECEIVE_PORT = 9890;
constexpr int YOLOMSG_RECEIVE_PORT = 9010;
constexpr int YOLOMSG_SEND_PORT = 9009;

constexpr const char IP_WSL2[] = "172.20.35.129";

constexpr int PM_MAX_ITER = 25;   // maximum number of iterations for computation
constexpr int PM_MAX_PERIOD = 10; // maximum period for computation in milliseconds
constexpr int ENCODER_RESOLUTION = 3840;
constexpr double CONTROL_CYCLE = 0.025;
constexpr double STEP_SCALE = 0.04;
constexpr double LASER_RANGE = 360.0;
constexpr int LASER_RAYS = 720;

// Real robot values
constexpr double A = 0.0965;
constexpr double B = 0.109;
constexpr double C = A + B;
constexpr double R = 0.065;
constexpr double OFFSET_X = 0.85e-2 - 1.45e-2; // LIDAR BODY FRAME OFFSET MEASURED FROM ROBOTS FRAME CM
constexpr double OFFSET_Y = 0.2e-2 - 0.65e-2;  // LIDAR BODY FRAME OFFSET MEASURE FROM ROBOTS FRAME CM

constexpr std::string_view FRONTCAM = "FrontCam";
constexpr std::string_view REARCAM = "RearCam";
constexpr std::string_view LEFTCAM = "LeftCam";
constexpr std::string_view RIGHTCAM = "RightCam";

// // Simulation values
// constexpr double A = 0.25 / 2 - 0.05;
// constexpr double B = 0.155 / 2 + 0.015;
// constexpr double C = A + B;
// constexpr double R = 0.065 / 2;

constexpr int SAFETY_THRESHOLD = 5;

enum class OperationalMode
{
    Online,
    Offline
};

constexpr double FX = 219.96470465;
constexpr double FY = 219.94273694;
constexpr double CX = 319.21197429;
constexpr double CY = 241.81387698;

#endif // AMR_PROJECT_CONFIG_H
