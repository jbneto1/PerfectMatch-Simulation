// SimTwoInterface.h

#ifndef PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
#define PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H

#include <tuple>
#include <array>
#include <asio.hpp>
#include <thread>
#include <Logger/logger.h>
#include <config/config.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <sstream>
#include <Localization/Localization.h>
#include <chrono>

class SimTwoInterface {
public:
    using DataCallback = std::function<void(const std::string &)>;

    SimTwoInterface(Logger &logger, Localization &localization, AMRController &controller);

    ~SimTwoInterface();

    void registerCallback(DataCallback callback);

    std::tuple<std::array<int, 4>, Pose, std::array<LaserPoint, 720>, bool>
    getSensorData(const std::string &data);

    void sendWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed);

    void startReceive();

private:
    void handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/);

    asio::io_context io_context;
    asio::ip::udp::socket socket;
    asio::ip::udp::endpoint sender_endpoint;
    std::array<char, MAX_BUFFER_SIZE> recv_buffer;
    std::thread thread;
    DataCallback dataCallback;
    Logger &logger;
    Localization &localization;
    AMRController &controller;
};

#endif //PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
