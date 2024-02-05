#include "SimTwoInterface.h"

// Constructor
SimTwoInterface::SimTwoInterface(Logger &logger, Localization &localization, AMRController &controller)
    : sim_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), SIMTWO_RECEIVE_PORT)),
      logger(logger), localization(localization), controller(controller), sync_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), SYNCMSG_RECEIVE_PORT)),
      yolo_socket(yoloIoContext, asio::ip::udp::endpoint(asio::ip::udp::v4(), YOLOMSG_RECEIVE_PORT))
{
    startLogging = false;
    earlyStop = false;
    run = true;
    std::fill(simRecvBuffer.begin(), simRecvBuffer.end(), 0);
    std::fill(yoloRecvBuffer.begin(), yoloRecvBuffer.end(), 0);
    logger.info("Simulator Interface created and listening for data.");
    logger.info("Listening for UDP datagrams on port: " + std::to_string(SIMTWO_RECEIVE_PORT));
}

// Destructor
SimTwoInterface::~SimTwoInterface()
{
    // Ensure threads are joined before destruction
    if (yoloThread.joinable())
    {
        yoloThread.join();
    }
}

void SimTwoInterface::runIoContextReadyMsg()
{
    asio::executor_work_guard<asio::io_context::executor_type> guard = asio::make_work_guard(io_context);
    waitForReadyMessage(); // Wait for ready message before starting
    while (!startLogging && !earlyStop)
    {
        io_context.run_one(); // Process one ASIO event (waiting for "ready" message)
    }
}

void SimTwoInterface::runIoContext()
{

    if (!earlyStop)
    {
        logger.debug("Waiting for the simulator.");
        io_context.run(); // Continue with the normal operation after receiving the message
    }
}

void SimTwoInterface::runYoloIoContext()
{
    asio::executor_work_guard<asio::io_context::executor_type> yoloGuard = asio::make_work_guard(yoloIoContext);
    startYoloReceive();
    yoloIoContext.run();
}

// Register a callback function for when data is received
void SimTwoInterface::registerCallback(DataCallback callback)
{
    dataCallback = std::move(callback);
    logger.debug("Data callback registered.");
}

// Start receiving data
void SimTwoInterface::startSimReceive()
{

    logFrequencySimTwo();

    sim_socket.async_receive_from(
        asio::buffer(simRecvBuffer),
        sender_endpoint,
        [this](std::error_code ec, std::size_t bytes_received)
        {
            handleReceive(ec, bytes_received);
            memset(simRecvBuffer.data(), 0, bytes_received);
            if (run)
            {
                this->startSimReceive();
            }
        });
}

void SimTwoInterface::startYoloReceive()
{

    logFrequencyYOLO();

    yolo_socket.async_receive_from(
        asio::buffer(yoloRecvBuffer),
        sender_endpoint,
        [this](const asio::error_code &ec, std::size_t bytes_received)
        {
            if (!ec)
            {
                std::lock_guard<std::mutex> guard(yoloDataMutex);
                bufferYoloData.assign(yoloRecvBuffer.data(), bytes_received);
            }
            startYoloReceive(); // Continue receiving
        });
}

// Implement getLatestYoloData
std::string SimTwoInterface::getLatestYoloData()
{
    std::lock_guard<std::mutex> guard(yoloDataMutex);
    return bufferYoloData;
}

void SimTwoInterface::waitForReadyMessage()
{
    std::array<char, MAX_BUFFER_SIZE> recv_buffer;

    // Function to setup receiving the ready message
    std::function<void()> setup_receive;
    setup_receive = [this, &recv_buffer, &setup_receive]()
    {
        std::fill(recv_buffer.begin(), recv_buffer.end(), 0);
        sync_socket.async_receive_from(
            asio::buffer(recv_buffer), sender_endpoint,
            [this, &recv_buffer, &setup_receive](std::error_code ec, std::size_t bytes_received)
            {
                if (!ec && bytes_received > 0)
                {
                    std::string message(recv_buffer.data(), bytes_received);
                    if (message == "ready")
                    {
                        this->logger.info("Received ready message. Sending acknowledgment.");
                        this->sendAckMessage(); // Send acknowledgment
                        this->startLogging = true;
                        startSimReceive();
                        yoloThread = std::thread(&SimTwoInterface::runYoloIoContext, this);
                        if (sync_socket.is_open())
                        {
                            asio::error_code ec;

                            sync_socket.close(ec);
                            if (ec)
                            {
                                logger.error("Error while closing socket: " + ec.message());
                            }
                        }
                    }
                    else
                    {
                        this->logger.warn("Unexpected message received: " + message);
                        setup_receive(); // Re-setup the async receive
                    }
                }
                else
                {
                    this->logger.error("Error in receiving ready message: " + ec.message());
                    setup_receive(); // Re-setup the async receive in case of error
                }
            });
    };

    setup_receive(); // Initial setup
}

void SimTwoInterface::handleReceive(const asio::error_code &error, std::size_t /*bytes_transferred*/)
{
    if (!error)
    {
        this->logger.trace("Received data without error. Handler called.");
        if (dataCallback)
        {
            dataCallback(std::string(simRecvBuffer.data()));
        }
    }
    else
    {
        this->logger.error("Error while receiving data: " + error.message());
    }
}

// Send wheel speeds
void SimTwoInterface::sendWheelSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
                                      double backRightSpeed)
{
    // Placeholder implementation, replace with actual logic
    logger.info("Setting wheel speeds: "
                "Front Left: " +
                std::to_string(frontLeftSpeed) +
                ", Front Right: " + std::to_string(frontRightSpeed) +
                ", Back Left: " + std::to_string(backLeftSpeed) +
                ", Back Right: " + std::to_string(backRightSpeed));
}

// Parse received data
std::tuple<std::array<int, 4>, Pose, std::optional<std::array<LaserPoint, 720>>>
SimTwoInterface::getSensorData(const std::string &data)
{
    std::istringstream iss(data);
    std::string line;
    std::array<int, 4> encoders{}; // encs (1..4) (FL, FR, BL, BR)
    std::array<double, 3> pose{};  // Pose (X, Y, Theta)
    std::optional<std::array<LaserPoint, 720>> lidar = std::nullopt;

    int encoder_index = 0;
    int pose_index = 0;

    logger.trace("Starting to parse sensor data...");

    while (std::getline(iss, line))
    {
        if (line.find("Enc") != std::string::npos)
        {
            std::getline(iss, line);
            encoders[encoder_index++] = std::stoi(line);
        }
        else if (line.find("X") != std::string::npos || line.find("Y") != std::string::npos ||
                 line.find("THETA") != std::string::npos)
        {
            std::getline(iss, line);
            pose[pose_index++] = std::stod(line);
        }
        else if (line.find("lidar") != std::string::npos)
        {
            // Initialize the lidar data if not done before
            if (!lidar)
            {
                lidar = std::array<LaserPoint, 720>{};
            }

            std::getline(iss, line);
            std::istringstream iss_lidar(line);
            std::string val;

            int lidar_index = 0;
            while (std::getline(iss_lidar, val, ','))
            {
                lidar.value()[lidar_index++].setD(std::stod(val));
            }
        }
    }

    Pose tmp = Pose();
    tmp = pose;

    return std::make_tuple(encoders, tmp, lidar);
}

void SimTwoInterface::stopIosContexts()
{
    logger.trace("Stopping I/O contexts and closing sockets..."); // Start of operation
    run = false;
    startLogging = false;
    earlyStop = true;

    io_context.restart();

    // Ensure the io_context isn't already stopped.
    if (!io_context.stopped())
    {
        io_context.stop();

        // Double-check if it's really stopped.
        if (io_context.stopped())
        {
            logger.debug("io_context has been successfully stopped.");
        }
        else
        {
            logger.error("Failed to stop io_context.");
        }
    }
    else
    {
        logger.warn("io_context was already stopped.");
    }

    if (sim_socket.is_open())
    {
        asio::error_code ec;

        sim_socket.close(ec);
        if (ec)
        {
            logger.error("Error while closing socket: " + ec.message());
        }
    }

    yoloIoContext.restart();

    if (!yoloIoContext.stopped())
    {
        // do the real stop
        yoloIoContext.stop();

        // Double-check if it's really stopped.
        if (yoloIoContext.stopped())
        {
            logger.debug("yoloIocontext has been successfully stopped.");
        }
        else
        {
            logger.error("Failed to stop yoloIocontext.");
        }
    }
    else
    {
        logger.warn("yoloIocontext was already stopped.");
    }

    if (yolo_socket.is_open())
    {
        asio::error_code ec;

        yolo_socket.close(ec);
        if (ec)
        {
            logger.error("Error while closing socket: " + ec.message());
        }
    }
    logger.trace("I/O contexts stopped and sockets closed."); // End of operation
}

asio::io_context &SimTwoInterface::getIoContext()
{
    return this->io_context;
}

void SimTwoInterface::sendAckMessage()
{
    const std::string ackMsg = "acknowledged";
    asio::ip::udp::endpoint receiver_endpoint(asio::ip::address::from_string(IP_WSL2), YOLOMSG_SEND_PORT);
    sync_socket.send_to(asio::buffer(ackMsg), receiver_endpoint);
}

void SimTwoInterface::logFrequencySimTwo()
{

    auto now = std::chrono::steady_clock::now();

    if (lastTime_simtwo != std::chrono::steady_clock::time_point{})
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime_simtwo);
        double freq = 1E6 / double(duration.count());
        localization.setFreq(freq);
        logger.info("SimTwo Comm[Hz]: " + formatWithTwoDecimals(freq));
    }
    lastTime_simtwo = now;
}

void SimTwoInterface::logFrequencyYOLO()
{

    auto now = std::chrono::steady_clock::now();

    if (lastTime_yolo != std::chrono::steady_clock::time_point{})
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime_yolo);
        double freq = 1E6 / double(duration.count());
        logger.debug("YOLO Comm[Hz]: " + formatWithTwoDecimals(freq));
    }
    lastTime_yolo = now;
}
