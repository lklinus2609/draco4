/**
 * @file test_simple_velocity.cpp
 * @brief Simple test for corrected velocity scaling
 */

#include "jd8_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <iomanip>

volatile bool running = true;
void signal_handler(int) { running = false; }

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    std::string interface_name = argv[1];
    
    std::cout << "=== SIMPLE VELOCITY TEST ===" << std::endl;
    std::cout << "Testing corrected velocity scaling" << std::endl;
    
    jd8::JD8Controller motor;
    
    if (!motor.initialize(interface_name) ||
        !motor.scan_network() ||
        !motor.configure_slaves() ||
        !motor.start_operation() ||
        !motor.enable_motor()) {
        std::cerr << "Motor initialization failed" << std::endl;
        return 1;
    }
    
    std::cout << "Motor initialized successfully" << std::endl;
    
    // Load and upload new configuration
    std::cout << "\nLoading new motor configuration..." << std::endl;
    if (!motor.load_configuration("../config/JDLINK8_config_file.csv")) {
        std::cerr << "Failed to load configuration file" << std::endl;
        return 1;
    }
    
    std::cout << "Uploading configuration to motor..." << std::endl;
    if (!motor.upload_configuration()) {
        std::cerr << "Failed to upload configuration to motor" << std::endl;
        return 1;
    }
    
    std::cout << "New configuration uploaded successfully" << std::endl;
    
    // Test 157.5 RPM
    std::cout << "\nTesting 157.5 RPM..." << std::endl;
    int32_t start_pos = motor.get_actual_position_counts();
    motor.set_velocity_rpm(157.5);
    
    // Start timing measurement
    auto start_time = std::chrono::steady_clock::now();
    
    double total_update_time = 0.0;
    
    // Precise 250Hz timing - sync with EtherCAT cycles
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 2500 && running; i++) { // 2500 cycles * 4ms = 10000ms = 10 seconds
        auto update_start = std::chrono::steady_clock::now();
        motor.update();
        auto update_end = std::chrono::steady_clock::now();
        
        total_update_time += std::chrono::duration_cast<std::chrono::microseconds>(update_end - update_start).count() / 1000.0;
        
        if (i % 250 == 0) { // Every 250 cycles = 1000ms at 250Hz
            int32_t current_pos = motor.get_actual_position_counts();
            double current_vel = motor.get_actual_velocity_rpm_precise();
            std::cout << "t=" << (i * 4) << "ms: " << current_vel << " RPM, pos=" 
                      << current_pos << " (change: " << (current_pos - start_pos) << ")" << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    // End timing measurement
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    int32_t end_pos = motor.get_actual_position_counts();
    int32_t position_change = end_pos - start_pos;
    double actual_rpm = jd8::JD8Constants::output_rpm_from_position_change(position_change, elapsed_ms / 1000.0);
    
    std::cout << "Results:" << std::endl;
    std::cout << "  Expected duration: 10000ms" << std::endl;
    std::cout << "  Actual duration: " << elapsed_ms << "ms" << std::endl;
    std::cout << "  Timing accuracy: " << std::fixed << std::setprecision(1) 
              << (100.0 * elapsed_ms / 10000.0) << "% of expected" << std::endl;
    std::cout << "  motor.update() total time: " << std::fixed << std::setprecision(1) << total_update_time << "ms" << std::endl;
    std::cout << "  motor.update() avg per cycle: " << std::fixed << std::setprecision(2) << (total_update_time / 2500.0) << "ms" << std::endl;
    std::cout << "  sleep_until total time: " << std::fixed << std::setprecision(1) << (2500 * 4.0) << "ms" << std::endl;
    std::cout << "  Overhead/other: " << std::fixed << std::setprecision(1) << (elapsed_ms - total_update_time - 10000.0) << "ms" << std::endl;
    std::cout << "  Position change: " << (end_pos - start_pos) << " counts" << std::endl;
    std::cout << "  Calculated RPM: " << std::fixed << std::setprecision(2) << actual_rpm << std::endl;
    std::cout << "  Velocity reading: " << std::fixed << std::setprecision(2) << motor.get_actual_velocity_rpm_precise() << " RPM" << std::endl;
    
    if (std::abs(actual_rpm - 157.5) < 10.0) {
        std::cout << "SUCCESS: Motor moving at correct speed" << std::endl;
    }
    
    motor.set_velocity_rpm(0);
    motor.disable_motor();
    motor.stop_operation();
    
    return 0;
}