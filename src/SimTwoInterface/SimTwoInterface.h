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
#include <optional>

class SimTwoInterface {
public:
    using DataCallback = std::function<void(const std::string &)>;

    // Constructors and Destructors
    SimTwoInterface(Logger &logger, Localization &localization, AMRController &controller);
    ~SimTwoInterface();

    // Sensor Data Processing
    std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>>
    getSensorData(const std::string &data);

    // IO Operations
    void runIoContext();
    void stopIoContext();
    asio::io_context &getIoContext();
    std::promise<void> exit_signal;

    // Network Communication
    void sendWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed);
    void registerCallback(DataCallback callback);
private:

    void handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/);
    void startReceive();
    void startReceiveInStrand();

    // Class Attributes

    asio::io_context io_context;
    asio::ip::udp::socket socket;
    asio::ip::udp::endpoint sender_endpoint;
    std::array<char, MAX_BUFFER_SIZE> recv_buffer;
    DataCallback dataCallback;
    Logger &logger;
    Localization &localization;
    AMRController &controller;
    bool run;
    asio::strand<asio::io_context::executor_type> strand;
    asio::executor_work_guard<asio::io_context::executor_type> guard = asio::make_work_guard(io_context);


};

#endif //PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
