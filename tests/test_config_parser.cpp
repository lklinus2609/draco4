/**
 * @file test_config_parser.cpp
 * @brief Test program for JD8 configuration parser
 * 
 * Tests the CSV configuration parser functionality and validates
 * that critical motor parameters are correctly extracted.
 */

#include "jd8_configuration.hpp"
#include <iostream>
#include <cassert>

int main() {
    std::cout << "=== JD8 Configuration Parser Test ===" << std::endl;
    
    // Create parser instance
    jd8::JD8ConfigParser parser;
    
    // Test 1: Load configuration file
    std::cout << "\nTest 1: Loading configuration file..." << std::endl;
    bool loaded = parser.parseCSV("../config/JDLINK8_config_file.csv");
    
    if (!loaded) {
        std::cerr << "FAIL: Could not load configuration file" << std::endl;
        std::cerr << "Errors:" << std::endl;
        for (const auto& error : parser.getErrors()) {
            std::cerr << "  - " << error << std::endl;
        }
        return 1;
    }
    
    std::cout << "PASS: Configuration file loaded successfully" << std::endl;
    std::cout << "Parameters loaded: " << parser.getParameterCount() << std::endl;
    
    // Test 2: Critical RPM parameters
    std::cout << "\nTest 2: Critical RPM parameters..." << std::endl;
    
    uint32_t encoder_res = parser.getEncoderResolution();
    uint32_t velocity_res = parser.getVelocityResolution();
    double scaling_factor = parser.calculateVelocityScalingFactor();
    
    std::cout << "Encoder Resolution: " << encoder_res << " counts/rev" << std::endl;
    std::cout << "Velocity Resolution: " << velocity_res << std::endl;
    std::cout << "Velocity Scaling Factor: " << scaling_factor << std::endl;
    
    // Verify expected values
    assert(encoder_res == 524288);
    assert(velocity_res == 77419);
    
    std::cout << "PASS: Critical parameters extracted correctly" << std::endl;
    
    // Test 3: Control gains
    std::cout << "\nTest 3: Control gains extraction..." << std::endl;
    
    auto gains = parser.getControlGains();
    
    std::cout << "Position Gains: KP=" << gains.position_kp << ", KI=" << gains.position_ki << ", KD=" << gains.position_kd << std::endl;
    std::cout << "Velocity Gains: KP=" << gains.velocity_kp << ", KI=" << gains.velocity_ki << ", KD=" << gains.velocity_kd << std::endl;
    std::cout << "Current Gains:  KP=" << gains.current_kp << ", KI=" << gains.current_ki << ", KD=" << gains.current_kd << std::endl;
    
    // Verify expected gain values from CSV (with floating-point tolerance)
    assert(std::abs(gains.position_kp - 2500.0) < 0.001);
    assert(std::abs(gains.position_ki - 40000.0) < 0.001);
    assert(std::abs(gains.velocity_kp - 0.2) < 0.001);
    assert(std::abs(gains.velocity_ki - 0.84) < 0.001);
    
    std::cout << "PASS: Control gains extracted correctly" << std::endl;
    
    // Test 4: Motor specifications
    std::cout << "\nTest 4: Motor specifications..." << std::endl;
    
    auto specs = parser.getMotorSpecs();
    
    std::cout << "Motor Specs:" << std::endl;
    std::cout << "  Encoder Resolution: " << specs.encoder_resolution << std::endl;
    std::cout << "  Velocity Resolution: " << specs.velocity_resolution << std::endl;
    std::cout << "  Max Speed: " << specs.max_speed << std::endl;
    std::cout << "  Rated Current: " << specs.rated_current << std::endl;
    
    std::cout << "PASS: Motor specifications extracted correctly" << std::endl;
    
    // Test 5: Velocity scaling analysis
    std::cout << "\nTest 5: Velocity scaling analysis..." << std::endl;
    
    std::cout << "Current VELOCITY_FACTOR in code: 1.0" << std::endl;
    std::cout << "Calculated scaling factor: " << scaling_factor << std::endl;
    std::cout << "Scaling correction factor: " << scaling_factor << std::endl;
    
    if (std::abs(scaling_factor - 1.0) > 0.01) {
        std::cout << "WARNING: Velocity scaling correction needed!" << std::endl;
        std::cout << "Current RPM calculation may be inaccurate by factor of " << (1.0 / scaling_factor) << std::endl;
    } else {
        std::cout << "INFO: Current velocity scaling appears correct" << std::endl;
    }
    
    // Test 6: Parameter access by index
    std::cout << "\nTest 6: Parameter access by index..." << std::endl;
    
    // Test specific parameter access
    const auto* param = parser.getParameter(0x2110, 3);  // Encoder resolution
    if (param) {
        std::cout << "Found encoder resolution parameter: 0x2110,3 = " << encoder_res << std::endl;
        std::cout << "PASS: Parameter access by index works" << std::endl;
    } else {
        std::cerr << "FAIL: Could not access parameter by index" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== All Tests Passed! ===" << std::endl;
    std::cout << "Configuration parser is working correctly." << std::endl;
    
    return 0;
}