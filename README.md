# Draco4 EtherCAT Motor Controller

A C++ library for controlling JD8 servo motors via EtherCAT communication using the SOEM (Simple Open EtherCAT Master) library.

## Features

- **CIA402 State Machine**: Complete implementation of the CIA402 motor control state machine
- **Multiple Control Modes**: Support for velocity, position, and torque control
- **Safety Features**: Torque ramping, position rate limiting, and fault recovery
- **Real-time Operation**: Designed for 1ms cycle time real-time control
- **Professional Documentation**: Comprehensive Doxygen-style API documentation

## Requirements

- CMake 3.16 or higher
- C++17 compatible compiler (GCC, Clang)
- SOEM library (system-installed)
- Root privileges for EtherCAT network access

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Basic Initialization

```cpp
#include "jd8_controller.hpp"

jd8::JD8Controller motor;

// Initialize EtherCAT
motor.initialize("eth0");  // Replace with your network interface
motor.scan_network();
motor.configure_slaves();
motor.start_operation();

// Enable motor
motor.enable_motor();

// Control the motor
motor.set_velocity_rpm(1000);  // 1000 RPM

// Update loop (call at 1ms intervals)
while (running) {
    motor.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
```

### Test Programs

The project includes several test programs:

- `test_jd8_init` - Basic initialization test
- `test_jd8_velocity` - Velocity control test
- `test_jd8_position` - Position control test
- `test_jd8_torque` - Torque control test
- `test_jd8_integration` - Integration test

Run with:
```bash
sudo ./test_jd8_velocity eth0  # Replace with your interface
```

## Project Structure

```
draco4/
├── include/              # Header files
│   ├── jd8_controller.hpp
│   └── jd8_pdo_structures.hpp
├── src/                  # Implementation files
│   ├── jd8_controller.cpp
│   ├── jd8_configuration.cpp
│   └── ethercat_master.cpp
├── tests/                # Test programs
├── legacy/               # Legacy C reference implementation
├── config/               # Motor configuration files
└── CMakeLists.txt
```

## Safety Notes

- **Root Privileges**: EtherCAT communication requires root privileges
- **Network Interface**: Ensure the specified network interface is connected to your EtherCAT network
- **Motor Safety**: The library includes safety limits for torque and position changes
- **Emergency Stop**: Use `emergency_stop()` method for immediate motion halt

## Contributing

This is a motor control project focused on industrial automation. Please ensure any contributions maintain safety and real-time performance requirements.

## License

[Add your license information here]