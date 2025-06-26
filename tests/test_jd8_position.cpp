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
    
    std::cout << "=== Position Control Test (90 Degree Output Shaft Move) ===" << std::endl;
    
    int32_t start_output_pos = motor.get_actual_output_shaft_position_counts();
    int32_t start_motor_pos = motor.get_actual_position_counts();
    std::cout << "Starting output shaft position: " << start_output_pos << std::endl;
    std::cout << "Starting motor shaft position: " << start_motor_pos << std::endl;
    
    // Move output shaft by 90 degrees (1/4 revolution)
    int32_t target_offset = jd8::JD8Constants::COUNTS_PER_REV / 4;  // 90 degrees = 131072 counts
    int32_t target_pos = start_output_pos + target_offset;
    
    std::cout << "Target output shaft position: " << target_pos << " (change: +" << target_offset << " counts = 90 degrees)" << std::endl;
    std::cout << "Expected motor shaft movement: " << (target_offset * 7.75) << " counts" << std::endl;
    
    motor.set_position_counts(target_pos);
    
    // Precise 250Hz timing
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 400; i++) {
        motor.update();
        
        if (i % 50 == 0) {
            int32_t motor_shaft_pos = motor.get_actual_position_counts();
            int32_t output_shaft_pos = motor.get_actual_output_shaft_position_counts();
            int32_t output_error = target_pos - output_shaft_pos;
            
            std::cout << "Cycle " << i 
                      << " | Motor: " << motor_shaft_pos 
                      << " | Output: " << output_shaft_pos
                      << " | Target: " << target_pos
                      << " | Error: " << output_error << " counts" << std::endl;
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
