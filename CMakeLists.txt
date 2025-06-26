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

# Core functionality tests


if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_position.cpp)
    add_executable(test_jd8_position tests/test_jd8_position.cpp)
    target_link_libraries(test_jd8_position draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_torque.cpp)
    add_executable(test_jd8_torque tests/test_jd8_torque.cpp)
    target_link_libraries(test_jd8_torque draco4_ethercat)
endif()




# SDO integration test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sdo_integration.cpp)
    add_executable(test_sdo_integration tests/test_sdo_integration.cpp)
    target_link_libraries(test_sdo_integration draco4_ethercat)
endif()



# Simple velocity test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simple_velocity.cpp)
    add_executable(test_simple_velocity tests/test_simple_velocity.cpp)
    target_link_libraries(test_simple_velocity draco4_ethercat)
endif()


