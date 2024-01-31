// SimTwoInterface.h

#ifndef PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
#define PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H

#include <tuple>
#include <array>
#include <asio.hpp>
#include <thread>
#include "Logger/logger.h"
#include "config/config.h"
#include <iostream>
#include <iterator>
#include <algorithm>
#include <sstream>
#include "Localization/Localization.h"
#include "AMRController/AMRController.h"
#include <chrono>
#include <optional>
#include "utils/utils.h"

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
    void runIoContextReadyMsg();
    void stopIosContexts();
    asio::io_context &getIoContext();
    
    std::string getLatestYoloData();
    
    // Network Communication
    void sendWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed);
    void registerCallback(DataCallback callback);

private:

    void waitForReadyMessage();
    void sendAckMessage();

    void handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/);
    void startSimReceive();

    void runYoloIoContext();
    void startYoloReceive();

    // Class Attributes

    std::string bufferYoloData;
    std::mutex yoloDataMutex; // For thread-safe access to latestYoloData
    std::array<char, MAX_BUFFER_SIZE> yoloRecvBuffer;
    // Separate io_context and threads for YOLO data reception
    asio::io_context yoloIoContext;
    std::thread yoloThread;
    asio::ip::udp::socket yolo_socket;


    asio::io_context io_context;
    asio::ip::udp::socket sim_socket;
    asio::ip::udp::socket sync_socket;
    asio::ip::udp::endpoint sender_endpoint;
    std::array<char, MAX_BUFFER_SIZE> simRecvBuffer;
    DataCallback dataCallback;
    bool startLogging;
    bool earlyStop;
    Logger &logger;
    Localization &localization;
    AMRController &controller;
    bool run;


    void logFrequencySimTwo();
    void logFrequencyYOLO();
    std::chrono::steady_clock::time_point lastTime_simtwo;
    std::chrono::steady_clock::time_point lastTime_yolo;
};

#endif //PERFECTMATCH_SIMULATION_SIMTWOINTERFACE_H
