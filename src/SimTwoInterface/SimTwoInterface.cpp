// SimTwoInterface.cpp

#include "SimTwoInterface.h"

// Constructor
SimTwoInterface::SimTwoInterface(Logger &logger, Localization &localization, AMRController &controller)
        : socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), SIMTWO_RECEIVE_PORT)),
          logger(logger), localization(localization), controller(controller),
          strand(io_context.get_executor()) {
    run = true;
    startReceive();
    logger.info("Simulator Interface created and listening for data.");
    logger.info("Listening for UDP datagrams on port: " + std::to_string(SIMTWO_RECEIVE_PORT));
}

// Destructor
SimTwoInterface::~SimTwoInterface() {
}

void SimTwoInterface::runIoContext() {
    io_context.run();
}

// Register a callback function for when data is received
void SimTwoInterface::registerCallback(DataCallback callback) {
    dataCallback = std::move(callback);
    logger.debug("Data callback registered.");
}

// Start receiving data
void SimTwoInterface::startReceive() {

    auto now = std::chrono::steady_clock::now();

    // if this is not the first call, compute the frequency
    if (lastTime != std::chrono::steady_clock::time_point{}) {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime); //microseconds precision
        double freq = 1E6 / double(duration.count()); // freq = 1 / time_interval
        localization.setFreq(freq);
        std::cout << "freq: " << freq << '\n';
    }

    // save the call time for the next frequency computation
    lastTime = now;

    asio::post(strand, [&] { startReceiveInStrand(); });
}

void SimTwoInterface::startReceiveInStrand() {
    // Fill the receive buffer with zeros.
    std::fill(recv_buffer.begin(), recv_buffer.end(), 0);

    socket.async_receive_from(
            asio::buffer(recv_buffer),
            sender_endpoint,
            [this](std::error_code ec, std::size_t bytes_received) {
                handleReceive(ec, bytes_received);
            }
    );
}

void SimTwoInterface::handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/) {
    if (!error) {
        this->logger.trace("Received data without error. Handler called.");
        if (dataCallback) {
            dataCallback(std::string(recv_buffer.data()));
        }

        // Set up to receive more data
        if (run) {
            this->startReceive();
        }
    } else {
        this->logger.error("Error while receiving data: " + error.message());
    }
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
std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>>
SimTwoInterface::getSensorData(const std::string &data) {
    std::istringstream iss(data);
    std::string line;
    std::array<int, 4> encoders{}; // encs (1..4) (FL, FR, BL, BR)
    std::array<double, 3> pose{};  // Pose (X, Y, Theta)
    std::optional<std::array<LaserPoint, 720>> lidar = std::nullopt;

    int encoder_index = 0;
    int pose_index = 0;

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
            // Initialize the lidar data if not done before
            if (!lidar) {
                lidar = std::array<LaserPoint, 720>{};
            }

            std::getline(iss, line);
            std::istringstream iss_lidar(line);
            std::string val;

            int lidar_index = 0;
            while (std::getline(iss_lidar, val, ',')) {
                lidar.value()[lidar_index++].setD(std::stod(val));
            }
        }
    }

    Pose tmp = Pose();
    tmp = pose;

    return std::make_tuple(encoders, tmp, lidar);
}


void SimTwoInterface::stopIoContext() {
    logger.trace("Stopping I/O context...");  // Start of operation
    run = false;

    io_context.restart();

    // Ensure the io_context isn't already stopped.
    if (!io_context.stopped()) {
        // do the real stop
        guard.reset();
        io_context.stop();

        // Double-check if it's really stopped.
        if (io_context.stopped()) {
            logger.debug("io_context has been successfully stopped.");
        } else {
            logger.error("Failed to stop io_context.");
        }
    } else {
        logger.warn("io_context was already stopped.");
    }
    if (socket.is_open()) {
        asio::error_code ec;

        socket.close(ec);
        if (ec) {
            logger.error("Error while closing socket: " + ec.message());
        }
    }
    logger.trace("I/O context stopped.");  // End of operation
}


asio::io_context &SimTwoInterface::getIoContext() {
    return this->io_context;
}


