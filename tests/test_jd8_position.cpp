#include "jd8_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    jd8::JD8Controller motor;
    
    if (!motor.initialize(argv[1]) || !motor.scan_network() || 
        !motor.configure_slaves() || !motor.start_operation() || 
        !motor.enable_motor()) {
        std::cerr << "Initialization failed" << std::endl;
        return 1;
    }
    
    std::cout << "=== Position Control Test (Rate-Limit Safe) ===" << std::endl;
    
    int32_t start_pos = motor.get_actual_position_counts();
    std::cout << "Starting position: " << start_pos << std::endl;
    
    // Use small, safe position moves with increased rate limits
    int32_t target_offset = 50000;  // Should work with 10x increased limits
    int32_t target_pos = start_pos + target_offset;
    
    std::cout << "Target position: " << target_pos << " (change: +" << target_offset << " counts)" << std::endl;
    
    motor.set_position_counts(target_pos);
    
    // Precise 250Hz timing
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 750; i++) {
        motor.update();
        
        if (i % 50 == 0) {
            int32_t current_pos = motor.get_actual_position_counts();
            int32_t error = target_pos - current_pos;
            
            std::cout << "Cycle " << i 
                      << " | Position: " << current_pos 
                      << " | Target: " << target_pos
                      << " | Error: " << error << " counts" << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    motor.disable_motor();
    motor.stop_operation();
    
    std::cout << "\nPosition control test completed!" << std::endl;
    return 0;
}
