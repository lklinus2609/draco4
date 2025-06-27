# Draco4 EtherCAT Motor Controller

A C++ library for controlling JD8 servo motors via EtherCAT communication using the SOEM (Simple Open EtherCAT Master) library.

## Features

- **CIA402 State Machine**: Complete implementation of the CIA402 motor control state machine
- **Multiple Control Modes**: Support for velocity, position, and torque control
- **Safety Features**: Torque ramping, position rate limiting, and fault recovery
- **Real-time Operation**: Designed for 4ms cycle time (250Hz) real-time control
- **Professional Documentation**: Comprehensive Doxygen-style API documentation
- **Configuration Management**: CSV-based motor configuration with SDO parameter upload

## Requirements

- CMake 3.16 or higher
- C++17 compatible compiler (GCC, Clang)
- SOEM library (Simple Open EtherCAT Master)
- Root privileges for EtherCAT network access

### Installing SOEM

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install libsoem-dev
```

**From source:**
```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
mkdir build && cd build
cmake ..
make
sudo make install
```

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
motor.scan();
motor.configure();
motor.start();

// Load motor configuration
motor.loadConfig("config/JDLINK8_config_file.csv");
motor.uploadConfig();

// Enable motor
motor.enable();

// Control the motor
motor.set_velocity_rpm(1000);  // 1000 RPM

// Update loop (call at 4ms intervals for 250Hz)
while (running) {
    motor.update();
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
}
```

### Test Programs

The project includes several test programs:

- `test_jd8_velocity` - Velocity control with precision timing analysis
- `test_jd8_position` - Position control with gear ratio scaling  
- `test_jd8_torque` - Torque control test with safety limits
- `test_sdo_integration` - SDO and configuration integration test

Run with:
```bash
sudo ./test_jd8_velocity eth0  # Replace with your interface
```

## Key Functions

### Motor Control
- `initialize(interface)` - Initialize EtherCAT master
- `scan()` - Scan for EtherCAT slaves
- `configure()` - Configure discovered slaves
- `start()` - Start operational mode
- `enable()` - Enable motor using CIA402 state machine

### Configuration
- `loadConfig(file)` - Load motor parameters from CSV
- `uploadConfig()` - Upload configuration to motor via SDO

### Control Commands
- `set_velocity_rpm(rpm)` - Set velocity command
- `setPosition(counts)` - Set position command
- `setTorque(millinm)` - Set torque command

### Feedback
- `getMotorRPM()` - Get motor shaft velocity
- `getOutputRPM()` - Get output shaft velocity  
- `getPosition()` - Get motor shaft position
- `getOutputPos()` - Get output shaft position
- `getTorque()` - Get actual torque feedback

## Safety Notes

- **Root Privileges**: EtherCAT communication requires root privileges
- **Network Interface**: Ensure the specified network interface is connected to your EtherCAT network
- **Motor Safety**: The library includes safety limits for torque and position changes
- **Emergency Stop**: Use `eStop()` method for immediate motion halt
- **Real-time Timing**: Maintain 4ms (250Hz) update cycles for optimal performance

## Contributing

This is a motor control project focused on industrial automation. Please ensure any contributions maintain safety and real-time performance requirements.

## License

[Add your license information here]