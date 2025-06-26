# =====================================
# Draco4 EtherCAT Motor Controller
# =====================================
# 
# CMake build configuration for JD8 servo motor control system
# using EtherCAT communication via SOEM library.
#
# Requirements:
#   - CMake 3.16+
#   - SOEM library (system-installed)
#   - C++17 compiler support

cmake_minimum_required(VERSION 3.16)
project(draco4_ethercat 
    VERSION 1.0.0
    DESCRIPTION "JD8 EtherCAT Motor Controller"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SOEM (Simple Open EtherCAT Master) library
find_package(PkgConfig REQUIRED)
pkg_check_modules(SOEM REQUIRED soem)

# Include directories
include_directories(include)
include_directories(${SOEM_INCLUDE_DIRS})

# =====================================
# Core Library
# =====================================

add_library(draco4_ethercat
    src/ethercat_master.cpp
    src/jd8_controller.cpp
    src/jd8_configuration.cpp
    src/jd8_sdo_manager.cpp
)

target_link_libraries(draco4_ethercat 
    ${SOEM_LIBRARIES}
    pthread
)

# =====================================
# Legacy Reference Implementation
# =====================================

add_executable(draco_test_legacy legacy/draco_test.c)
target_include_directories(draco_test_legacy PRIVATE ${SOEM_INCLUDE_DIRS})
target_link_libraries(draco_test_legacy ${SOEM_LIBRARIES} pthread)

# =====================================
# Test Suite
# =====================================

# Basic functionality tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_init.cpp)
    add_executable(test_jd8_init tests/test_jd8_init.cpp)
    target_link_libraries(test_jd8_init draco4_ethercat)
endif()

# Control mode tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_velocity.cpp)
    add_executable(test_jd8_velocity tests/test_jd8_velocity.cpp)
    target_link_libraries(test_jd8_velocity draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_position.cpp)
    add_executable(test_jd8_position tests/test_jd8_position.cpp)
    target_link_libraries(test_jd8_position draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_torque.cpp)
    add_executable(test_jd8_torque tests/test_jd8_torque.cpp)
    target_link_libraries(test_jd8_torque draco4_ethercat)
endif()

# Integration tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_integration.cpp)
    add_executable(test_jd8_integration tests/test_jd8_integration.cpp)
    target_link_libraries(test_jd8_integration draco4_ethercat)
endif()

# Configuration parser test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_config_parser.cpp)
    add_executable(test_config_parser tests/test_config_parser.cpp)
    target_link_libraries(test_config_parser draco4_ethercat)
endif()

# Velocity scaling test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_velocity_scaling.cpp)
    add_executable(test_velocity_scaling tests/test_velocity_scaling.cpp)
    target_link_libraries(test_velocity_scaling draco4_ethercat)
endif()

# SDO integration test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sdo_integration.cpp)
    add_executable(test_sdo_integration tests/test_sdo_integration.cpp)
    target_link_libraries(test_sdo_integration draco4_ethercat)
endif()

# Hardware integration test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_hardware_integration.cpp)
    add_executable(test_hardware_integration tests/test_hardware_integration.cpp)
    target_link_libraries(test_hardware_integration draco4_ethercat)
endif()

# Velocity diagnostics test - REMOVED (development artifact)
# Velocity fix test - REMOVED (development artifact)

# Complete scaling fix test
# if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_complete_scaling_fix.cpp)
#     add_executable(test_complete_scaling_fix tests/test_complete_scaling_fix.cpp)
#     target_link_libraries(test_complete_scaling_fix draco4_ethercat)
# endif()

# Simple velocity test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simple_velocity.cpp)
    add_executable(test_simple_velocity tests/test_simple_velocity.cpp)
    target_link_libraries(test_simple_velocity draco4_ethercat)
endif()

# Encoder diagnostics test - REMOVED (development artifact)
