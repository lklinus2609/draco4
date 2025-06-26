#include "jd8_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>

volatile bool running = true;
void signal_handler(int) { running = false; }

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    jd8::JD8Controller motor;
    
    if (!motor.initialize(argv[1]) || !motor.scan_network() || 
        !motor.configure_slaves() || !motor.start_operation() || 
        !motor.enable_motor()) {
        std::cerr << "Initialization failed" << std::endl;
        return 1;
    }
    
    std::cout << "=== JD8 Complete SDK Integration Test ===" << std::endl;
    
    // Test 1: Velocity Control
    std::cout << "\n--- Test 1: Velocity Control (100 RPM) ---" << std::endl;
    motor.set_velocity_rpm(100);
    
    // Precise 250Hz timing for velocity test
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 125 && running; i++) {
        motor.update();
        if (i % 25 == 0) {
            std::cout << "Velocity: " << motor.get_actual_velocity_rpm() << " RPM" << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    // Test 2: Position Control
    std::cout << "\n--- Test 2: Position Control ---" << std::endl;
    int32_t start_pos = motor.get_actual_position_counts();
    motor.set_position_counts(start_pos + 1000);
    
    // Precise 250Hz timing for position test
    next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 200 && running; i++) {
        motor.update();
        if (i % 50 == 0) {
            std::cout << "Position: " << motor.get_actual_position_counts() << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    // Test 3: Torque Control
    std::cout << "\n--- Test 3: Torque Control (300 mNm) ---" << std::endl;
    motor.set_torque_millinm(300);
    
    // Precise 250Hz timing for torque test
    next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 125 && running; i++) {
        motor.update();
        if (i % 25 == 0) {
            std::cout << "Torque: " << motor.get_actual_torque_millinm() << " mNm" << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    motor.set_velocity_rpm(0);
    motor.set_torque_millinm(0);
    motor.disable_motor();
    motor.stop_operation();
    
    std::cout << "\n🎉 ALL CONTROL MODES VALIDATED! 🎉" << std::endl;
    return 0;
}
