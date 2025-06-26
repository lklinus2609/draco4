/**
 * @file test_jd8_init.cpp
 * @brief Initialization test for JD8 motor controller
 * 
 * Tests the basic initialization sequence: EtherCAT master init,
 * network scan, slave configuration, and operational state startup.
 */

#include "jd8_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface_name>" << std::endl;
        return 1;
    }
    
    jd8::JD8Controller controller;
    
    std::cout << "=== JD8 Initialization Test ===" << std::endl;
    
    if (!controller.initialize(argv[1])) {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }
    
    if (!controller.scan_network()) {
        std::cerr << "Network scan failed" << std::endl;
        return 1;
    }
    
    if (!controller.configure_slaves()) {
        std::cerr << "Configuration failed" << std::endl;
        return 1;
    }
    
    if (!controller.start_operation()) {
        std::cerr << "Operation start failed" << std::endl;
        return 1;
    }
    
    std::cout << "Initialization completed successfully!" << std::endl;
    controller.stop_operation();
    return 0;
}
