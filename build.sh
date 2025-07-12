#!/bin/bash

# ToyAR Build Script
# This script automates the build process for the ToyAR platform

echo "Building ToyAR - Augmented Reality Platform"
echo "=========================================="

# Check if build directory exists, create if not
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

echo "Running CMake configuration..."
cmake .. -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

echo "Building project..."
# Check if we have ninja or make
if command -v ninja &> /dev/null; then
    ninja
elif [ -f "Makefile" ]; then
    make -j$(nproc)
elif [ -f "build.ninja" ]; then
    ninja
else
    echo "No build system found (neither make nor ninja)"
    exit 1
fi

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build successful!"
echo "Run './toyar_demo' to start the AR application"
