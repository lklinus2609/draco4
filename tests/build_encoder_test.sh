#!/bin/bash

# Quick build script for encoder diagnostics test
# This script builds just the encoder test without requiring full cmake build

echo "=== Building Encoder Diagnostics Test ==="

# Check if we're in the right directory
if [ ! -f "test_encoder_diagnostics.cpp" ]; then
    echo "Error: Must run from tests directory"
    exit 1
fi

# Move to parent directory for build
cd ..

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure with cmake
echo "Configuring build..."
cmake .. || {
    echo "Error: CMake configuration failed"
    echo "Make sure SOEM library is installed:"
    echo "  sudo apt-get install libsoem-dev"
    exit 1
}

# Build just the encoder test
echo "Building encoder diagnostics test..."
make test_encoder_diagnostics || {
    echo "Error: Build failed"
    exit 1
}

echo "✓ Build successful!"
echo ""
echo "To run the test:"
echo "  sudo ./test_encoder_diagnostics <interface_name>"
echo "  (Example: sudo ./test_encoder_diagnostics enx3c18a042f97c)"
echo ""
echo "Note: Root privileges required for EtherCAT access"