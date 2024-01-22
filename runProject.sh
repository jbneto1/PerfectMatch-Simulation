#!/bin/bash

# Start C++ program
./build/PM_project &

# Start Python program
python3 srcPython/YOLOV8/main.py &
