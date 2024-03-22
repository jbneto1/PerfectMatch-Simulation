//
// Created by jabra on 6/5/2023.
//

#ifndef AMR_PROJECT_CONFIG_H
#define AMR_PROJECT_CONFIG_H

constexpr int MAX_BUFFER_SIZE = 16256;
constexpr int SIMTWO_RECEIVE_PORT = 9000;
constexpr int SYNCMSG_RECEIVE_PORT = 9890;
constexpr int YOLOMSG_RECEIVE_PORT = 9010;
constexpr int YOLOMSG_SEND_PORT = 9009;

const char *const IP_WSL2 = "172.20.35.129";

constexpr int PM_MAX_ITER = 25;   // maximum number of iterations for computation
constexpr int PM_MAX_PERIOD = 10; // maximum period for computation in milliseconds
constexpr int ENCODER_RESOLUTION = 3840;
constexpr double CONTROL_CYCLE = 0.025;
constexpr double STEP_SCALE = 0.04;
constexpr double LASER_RANGE = 360.0;
constexpr int LASER_RAYS = 720;
constexpr double A = 0.25 / 2 - 0.05;
constexpr double B = 0.155 / 2 + 0.015;
constexpr double C = A + B;
constexpr double R = 0.065 / 2;

constexpr int SAFETY_THRESHOLD = 5;

enum class OperationalMode
{
    Online,
    Offline
};

#endif // AMR_PROJECT_CONFIG_H
