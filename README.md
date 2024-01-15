# Perfect Match C++ Project

## Overview

The Perfect Match project is a C++ simulation system for localizing an autonomous mobile robot (AMR) within a virtual environment. It interfaces with a simulator, receives sensor data and process it, and visualizes the localization process. It also has the feature to log the data.

## Building the Project

### Prerequisites

Ensure you have the following installed:
- CMake version 3.22.1 or higher

Third-party libraries dependencies: eigen, gl3w, ImGUI, spdlog, standalone_ASIO, stb image loader

### Compilation Steps

```bash
mkdir build
cd build
cmake ..
make
