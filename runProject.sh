#!/bin/bash

# Store the root directory path
ROOT_DIR="$(dirname "$(readlink -f "$0")")"

# Function to kill background processes on exit
cleanup() {
    echo "$(date) - Stopping background processes..."
    kill $CPP_PID $PYTHON_PID 2>/dev/null
    echo "$(date) - Sent kill signal to C++ and Python processes."
    wait $CPP_PID 2>/dev/null
    echo "$(date) - C++ process stopped."
    wait $PYTHON_PID 2>/dev/null
    echo "$(date) - Python process stopped."
    echo "$(date) - All processes stopped."
}
# Set trap to call cleanup function when the script exits or is interrupted
trap cleanup INT

# Start the C++ program from the build directory
(cd "$ROOT_DIR/build" && ./PM_project) &
CPP_PID=$!
echo "$(date) - Started C++ program with PID $CPP_PID"

# Start the Python program from the YOLOV8 directory
(cd "$ROOT_DIR/srcPython/YOLOv8" && python3 main.py) &
PYTHON_PID=$!
echo "$(date) - Started Python program with PID $PYTHON_PID"

# Wait for both processes
wait $CPP_PID
wait $PYTHON_PID
