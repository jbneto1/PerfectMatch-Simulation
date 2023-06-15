// SimTwoInterface.cpp

#include "SimTwoInterface.h"

// Constructor
SimTwoInterface::SimTwoInterface(Logger &logger, Localization &localization, AMRController &controller)
        : socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), SIMTWO_RECEIVE_PORT)),
          logger(logger), localization(localization), controller(controller) {
    startReceive();
    thread = std::thread([this]() { io_context.run(); });
    logger.info("Simulator Interface created and listening for data.");
    logger.info("Listening for UDP datagrams on port: " + std::to_string(SIMTWO_RECEIVE_PORT));
}

// Destructor
SimTwoInterface::~SimTwoInterface() {
    io_context.stop();
    if (thread.joinable()) {
        thread.join();
    }
    logger.trace("Simulator Interface destroyed.");
}

// Register a callback function for when data is received
void SimTwoInterface::registerCallback(DataCallback callback) {
    dataCallback = std::move(callback);
    logger.debug("Data callback registered.");
}

// Start receiving data
void SimTwoInterface::startReceive() {
    std::fill(recv_buffer.begin(), recv_buffer.end(), 0);
    socket.async_receive_from(
            asio::buffer(recv_buffer), sender_endpoint,
            [this](std::error_code ec, std::size_t bytes_received) {
                handleReceive(ec, bytes_received);
            });
    logger.trace("Receiving started.");
}

// Handle data received from the socket
void SimTwoInterface::handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/) {
    if (!error) {
        logger.trace("Received data without error. Handler called.");
        if (dataCallback) {
            dataCallback(std::string(recv_buffer.data()));
        }
    } else {
        logger.error("Error while receiving data: " + error.message());
    }

    // Set up to receive more data
    startReceive();
}

// Send wheel speeds
void SimTwoInterface::sendWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
                                      double backRightSpeed) {
    // Placeholder implementation, replace with actual logic
    logger.info("Setting wheel speeds: "
                "Front Left: " + std::to_string(frontLeftSpeed) +
                ", Front Right: " + std::to_string(frontRightSpeed) +
                ", Back Left: " + std::to_string(backLeftSpeed) +
                ", Back Right: " + std::to_string(backRightSpeed));
}

// Parse received data
std::tuple<std::array<int, 4>, Pose, std::array<LaserPoint, 720>> SimTwoInterface::getSensorData(const std::string &data) {
    std::istringstream iss(data);
    std::string line;
    std::array<int, 4> encoders{}; // encs (1..4) (FL, FR, BL, BR)
    std::array<double, 3> pose{};  // Pose (X, Y, Theta)
    std::array<LaserPoint, 720> lidar{};

    int encoder_index = 0;
    int pose_index = 0;
    int lidar_index = 0;

    logger.trace("Starting to parse sensor data...");

    while (std::getline(iss, line)) {
        if (line.find("Enc") != std::string::npos) {
            std::getline(iss, line);
            encoders[encoder_index++] = std::stoi(line);
        } else if (line.find("X") != std::string::npos || line.find("Y") != std::string::npos ||
                   line.find("THETA") != std::string::npos) {
            std::getline(iss, line);
            pose[pose_index++] = std::stod(line);
        } else if (line.find("lidar") != std::string::npos) {
            std::getline(iss, line);
            std::istringstream iss_lidar(line);
            std::string val;
            while (std::getline(iss_lidar, val, ',')) {
                lidar[lidar_index++].setD(std::stod(val));
            }
        }
    }

    logger.info("Finished parsing sensor data. Encoders: (" +
                 std::to_string(encoders[0]) + ", " + std::to_string(encoders[1]) + ", " + std::to_string(encoders[2]) + ", " + std::to_string(encoders[3]) +
                 "), Pose: (" + std::to_string(pose[0]) + ", " + std::to_string(pose[1]) + ", " + std::to_string(pose[2]) +
                 "), Lidar points: " + std::to_string(lidar.size()));

    Pose tmp = Pose();
    tmp = pose;

    return std::make_tuple(encoders, tmp, lidar);
}
