# System Design

## Overview

This document provides a detailed description of the classes and their interactions within the Perfect Match project.

## Components Overview

- `SimTwoInterface`: Facilitates communication with the simulation environment.
- `Manager`: Orchestrates the operation of all components.
- `Localization`: Responsible for the robot's localization.
- `Visualizer`: Renders the localization process visually.
- `AMRController`: Intended to control the robot's navigation (to be implemented).
- `Logger`: Manages system-wide logging.

## Class Documentation

### Visualizer Class

The `Visualizer` class is integral to the graphical representation of the localization process. It uses OpenGL for rendering and ImGui for GUI elements to display real-time data.

#### Attributes
- `GLFWwindow* window`: A pointer to the GLFW window for rendering.
- `GLuint textureId`: OpenGL texture identifier for map visualization.
- `int texWidth, texHeight`: Dimensions of the texture.
- `std::atomic<bool> runRenderLoop`: Atomic flag to control the rendering loop.
- `VisualizationData visDataFront, visDataBack`: Double-buffer for storing visualization data to be rendered.

#### Methods
- `update`: Accepts the ground truth and estimated pose data, updating the visualization state.
- `render`: Handles the entire rendering process for each frame.
- `stop`: Signals the rendering loop to terminate.
- `cleanup`: Frees resources and safely terminates the rendering context.


### SimTwoInterface Class

The `SimTwoInterface` class manages communication with the simulation environment, sending control commands and receiving sensor data.

#### Attributes
- `asio::ip::udp::socket socket`: ASIO socket for UDP communication.
- `std::array<char, MAX_BUFFER_SIZE> recv_buffer`: Buffer for receiving data.
- `DataCallback dataCallback`: Callback function for handling incoming data.

#### Methods
- `getSensorData`: Extracts sensor information from the received data string.
- `runIoContext`: Starts the ASIO IO context to process asynchronous network operations.
- `stopIoContext`: Stops the IO context to end communication.
- `sendWheelSpeeds`: Sends velocity commands for the robot's wheels to the simulator.

### Manager Class

The `Manager` class acts as the central control unit, managing the lifecycle of all components and facilitating data flow.

#### Attributes
- `Localization localization`: Instance for managing localization algorithms.
- `SimTwoInterface interface`: Manages communication with the simulation environment.
- `Visualizer visualizer`: Handles visualization tasks.
- `std::thread visThread`: Thread dedicated to running the visualization loop.
- `asio::signal_set signals_`: ASIO signal set for handling system signals.

#### Methods
- `run`: Initializes all components and enters the main control loop.
- `stop`: Gracefully stops all operations and terminates the system.
- `onDataReceived`: Processes incoming data from the simulation environment.

### Logger Class

The `Logger` class utilizes the `spdlog` library to provide a versatile logging system.

#### Methods
- `trace`, `debug`, `info`, `warn`, `error`: Methods for logging messages at various severity levels.
- `fileLog_*`: Specialized logging methods for different data types and purposes.
- `set_level`: Adjusts the verbosity of the logging output.
- `setPattern`: Sets the format pattern for the log messages.
- `deactivate_Loggers`: Disables all logging outputs, useful for production environments where logging is not required.

#### Usage
- Usage Static method getInstance is used to retrieve a singleton instance configured at a specified log level.
- Logging methods are used throughout the project to record the flow of operations and significant events.

### Localization Class

The `Localization` class is responsible for determining the position of the AMR within the simulation environment, utilizing sensor data.

#### Attributes
- `ExtendedKalmanFilter EKF`: An object of the Extended Kalman Filter for state estimation.
- `PerfectMatch PM`: An instance of the Perfect Match algorithm for map matching.
- `Eigen::Matrix<double, 3, 4> forwardK_model`: A matrix representing the forward kinematics model of the robot.

#### Methods
- `processData_w_PM`: Processes encoder and lidar data using the Perfect Match algorithm.
- `processData_wo_PM`: Processes encoder data without map matching.
- `setPose`: Sets the initial pose of the robot.
- `getPose`: Retrieves the current estimated pose from the Kalman filter.
- `getFreq`: Returns the current operation frequency of the localization process.

### AMRController Class

The `AMRController` class is planned to control the robot's movement based on the estimated pose.

#### Attributes
- Currently, this class holds a reference to a `Logger` for logging purposes.

#### Methods
- `computeWheelSpeeds`: A placeholder for future implementation of wheel speed calculations based on the estimated pose.