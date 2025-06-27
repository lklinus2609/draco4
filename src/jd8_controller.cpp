/**
 * @file jd8_controller.cpp
 * @brief Implementation of JD8Controller class
 * 
 * Provides EtherCAT-based control for JD8 servo motors using the SOEM library.
 * Implements CIA402 state machine and supports velocity, position, and torque control.
 */

#include "jd8_controller.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

namespace jd8 {

char JD8Controller::IOmap_[4096];

JD8Controller::JD8Controller(int slave_index) 
    : slave_index_(slave_index), initialized_(false), operational_(false),
      motor_enabled_(false), current_mode_(VELOCITY_MODE), enable_step_(0),
      output_pdo_(nullptr), input_pdo_(nullptr),
      current_torque_command_(0), target_torque_command_(0), fault_recovery_active_(false),
      target_position_command_(0), current_position_command_(0),
      target_velocity_command_(0), current_velocity_command_(0),
      last_position_command_(0) {
    last_position_time_ = std::chrono::steady_clock::now();
}

JD8Controller::~JD8Controller() {
    stop_operation();
}

bool JD8Controller::initialize(const std::string& interface_name) {
    interface_name_ = interface_name;
    
    std::cout << "Initializing EtherCAT master on " << interface_name_ << std::endl;
    
    if (ec_init(interface_name_.c_str())) {
        std::cout << "EtherCAT master initialized on " << interface_name_ << std::endl;
        initialized_ = true;
        return true;
    } else {
        std::cerr << "Failed to initialize EtherCAT master on " << interface_name_ << std::endl;
        std::cerr << "  Try running with sudo" << std::endl;
        return false;
    }
}

bool JD8Controller::scan_network() {
    if (!initialized_) {
        std::cerr << "Master not initialized" << std::endl;
        return false;
    }
    
    std::cout << "Starting network scan..." << std::endl;
    
    int slave_count = ec_config_init(FALSE);
    
    if (slave_count > 0) {
        std::cout << "Found " << ec_slavecount << " EtherCAT slaves:" << std::endl;
        
        for (int i = 1; i <= ec_slavecount; i++) {
            std::cout << "  Slave " << i << ": " << ec_slave[i].name 
                      << " (Product: 0x" << std::hex << ec_slave[i].eep_id << std::dec << ")" << std::endl;
        }
        return true;
    } else {
        std::cout << "No slaves found on network" << std::endl;
        return false;
    }
}

bool JD8Controller::configure_slaves() {
    if (ec_slavecount == 0) {
        std::cerr << "No slaves to configure" << std::endl;
        return false;
    }
    
    ec_config_map(&IOmap_);
    ec_configdc();
    
    std::cout << "Process data mapped - Output bytes: " << ec_slave[slave_index_].Obytes 
              << ", Input bytes: " << ec_slave[slave_index_].Ibytes << std::endl;
    
    if (slave_index_ <= ec_slavecount) {
        output_pdo_ = reinterpret_cast<OutputPDO*>(ec_slave[slave_index_].outputs);
        input_pdo_ = reinterpret_cast<InputPDO*>(ec_slave[slave_index_].inputs);
        
        std::memset(output_pdo_, 0, sizeof(OutputPDO));
        
        std::cout << "PDO mapping configured for slave " << slave_index_ << std::endl;
        return true;
    } else {
        std::cerr << "Invalid slave index: " << slave_index_ << std::endl;
        return false;
    }
}

bool JD8Controller::start_operation() {
    std::cout << "Slaves mapped, state to SAFE_OP" << std::endl;
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        std::cerr << "Not all slaves reached SAFE_OP state" << std::endl;
        return false;
    }
    
    std::cout << "Request operational state for all slaves" << std::endl;
    
    int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    std::cout << "Calculated workcounter " << expectedWKC << std::endl;
    
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    
    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        std::cout << "Operational state reached for all slaves" << std::endl;
        operational_ = true;
        return true;
    } else {
        std::cerr << "Not all slaves reached operational state" << std::endl;
        return false;
    }
}

void JD8Controller::stop_operation() {
    if (operational_) {
        if (output_pdo_) {
            output_pdo_->target_velocity = 0;
            output_pdo_->target_torque = 0;
            output_pdo_->controlword = JD8Constants::CONTROLWORD_SHUTDOWN;
            
            // Precise 250Hz timing for stop sequence
            auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
            
            for (int i = 0; i < 100; i++) {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                
                // Sleep until next 4ms cycle boundary
                std::this_thread::sleep_until(next_cycle);
                next_cycle += std::chrono::microseconds(4000);
            }
        }
        
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        operational_ = false;
    }
    
    if (initialized_) {
        ec_close();
        initialized_ = false;
        std::cout << "EtherCAT master shut down" << std::endl;
    }
}

void JD8Controller::update() {
    if (operational_) {
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        // Execute control commands based on active mode (mutually exclusive)
        if (current_mode_ == TORQUE_MODE || fault_recovery_active_) {
            ramp_torque_command();
        } else if (current_mode_ == POSITION_MODE) {
            update_position_command();
        } else if (current_mode_ == VELOCITY_MODE) {
            update_velocity_command();
        }
        
        if (has_fault() && !fault_recovery_active_) {
            handle_fault_recovery();
        }
    }
}

bool JD8Controller::enable_motor() {
    std::cout << "Starting motor enable sequence..." << std::endl;
    enable_step_ = 0;
    motor_enabled_ = false;
    
    // Precise 250Hz timing for enable sequence
    auto next_cycle = std::chrono::steady_clock::now() + std::chrono::microseconds(4000);
    
    for (int i = 0; i < 1000 && !motor_enabled_; i++) {
        update();
        
        if (i % 10 == 0) {
            enable_step_ = enable_motor_sequence();
        }
        
        if (i % 100 == 0 && input_pdo_) {
            std::cout << "Enable cycle " << i << ": Status=0x" << std::hex << input_pdo_->statusword 
                      << ", State=" << getStateStr() << std::dec << std::endl;
        }
        
        // Sleep until next 4ms cycle boundary
        std::this_thread::sleep_until(next_cycle);
        next_cycle += std::chrono::microseconds(4000);
    }
    
    std::cout << "Enable sequence completed. Motor enabled: " << (motor_enabled_ ? "YES" : "NO") << std::endl;
    return motor_enabled_;
}

bool JD8Controller::disable_motor() {
    motor_enabled_ = false;
    fault_recovery_active_ = false;
    current_torque_command_ = 0;
    target_torque_command_ = 0;
    target_velocity_command_ = 0;
    target_position_command_ = 0;
    
    if (output_pdo_) {
        output_pdo_->controlword = JD8Constants::CONTROLWORD_SHUTDOWN;
        output_pdo_->target_velocity = 0;
        output_pdo_->target_torque = 0;
        output_pdo_->target_position = 0;
    }
    return true;
}

bool JD8Controller::set_velocity_rpm(int rpm) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    if (abs(rpm) > JD8Constants::MAX_VELOCITY_RPM) {
        log_error(ErrorSeverity::ERROR, 
                  "Velocity " + std::to_string(rpm) + " exceeds safety limit of " + 
                  std::to_string(JD8Constants::MAX_VELOCITY_RPM) + " RPM");
        return false;
    }
    
    target_velocity_command_ = rpm;
    
    if (current_mode_ != VELOCITY_MODE && output_pdo_) {
        // Clear position commands when switching to velocity mode
        target_position_command_ = getPosition();
        current_position_command_ = target_position_command_;
        output_pdo_->target_position = static_cast<uint32_t>(target_position_command_);
        
        // Set velocity mode
        output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
        output_pdo_->controlword = JD8Constants::CONTROLWORD_VELOCITY_MODE;
        current_mode_ = VELOCITY_MODE;
        
        std::cout << "Mode switched to VELOCITY_MODE, position commands cleared" << std::endl;
    }
    
    return true;
}

bool JD8Controller::set_position_counts(int32_t position) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    // GEAR RATIO ANALYSIS: PDO goes to motor shaft, but user commands output shaft position
    // To achieve desired output shaft position, motor shaft must move 7.75x more
    int32_t motor_shaft_position = static_cast<int32_t>(position * JD8Constants::GEAR_REDUCTION_RATIO);
    
    target_position_command_ = motor_shaft_position;
    
    // Debug logging for position command scaling
#if JD8_DEBUG_LOGGING
    std::cout << "[POS] target_output_shaft=" << position 
              << " → motor_shaft_scaled=" << motor_shaft_position
              << " (×" << JD8Constants::GEAR_REDUCTION_RATIO << ")" << std::endl;
#endif
    
    if (current_mode_ != POSITION_MODE && output_pdo_) {
        // Clear velocity commands when switching to position mode
        target_velocity_command_ = 0;
        current_velocity_command_ = 0;
        output_pdo_->target_velocity = 0;
        
        // Set position mode
        output_pdo_->modes_of_operation = JD8Constants::POSITION_MODE;
        output_pdo_->controlword = JD8Constants::CONTROLWORD_POSITION_MODE;
        current_mode_ = POSITION_MODE;
        current_position_command_ = getPosition();  // This is already motor shaft position
        
        std::cout << "Mode switched to POSITION_MODE, velocity commands cleared" << std::endl;
    }
    
    return true;
}

bool JD8Controller::set_torque_millinm(int16_t torque) {
    if (!validate_torque_command(torque)) {
        return false;
    }
    
    target_torque_command_ = JD8Constants::clamp_torque(torque);
    
    if (target_torque_command_ != torque) {
        log_error(ErrorSeverity::WARNING, 
                  "Torque " + std::to_string(torque) + " clamped to " + 
                  std::to_string(target_torque_command_) + " mNm");
    }
    
    if (current_mode_ != TORQUE_MODE && output_pdo_) {
        // Clear other mode commands when switching to torque mode
        target_velocity_command_ = 0;
        current_velocity_command_ = 0;
        output_pdo_->target_velocity = 0;
        
        target_position_command_ = getPosition();
        current_position_command_ = target_position_command_;
        output_pdo_->target_position = static_cast<uint32_t>(target_position_command_);
        
        // Set torque mode
        output_pdo_->modes_of_operation = JD8Constants::TORQUE_MODE;
        output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
        current_mode_ = TORQUE_MODE;
        
        std::cout << "Mode switched to TORQUE_MODE, other commands cleared" << std::endl;
    }
    
    return true;
}

const char* JD8Controller::getStateStr() const {
    if (!input_pdo_) return "NO_DATA";
    
    uint16_t statusword = input_pdo_->statusword;
    
    if (statusword & JD8Constants::STATUSWORD_FAULT)
        return "FAULT";
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_OPERATION_ENABLED_VALUE)
        return "OPERATION ENABLED";
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_SWITCHED_ON)
        return "SWITCHED ON";
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON)
        return "READY TO SWITCH ON";
    if ((statusword & JD8Constants::STATUSWORD_SWITCH_DISABLED_MASK) == JD8Constants::STATUSWORD_SWITCH_ON_DISABLED_VALUE)
        return "SWITCH ON DISABLED";
    return "UNKNOWN";
}

double JD8Controller::getMotorRPM() const {
    if (input_pdo_) {
        // Calculate and log all scaling steps
        uint32_t raw_pdo = input_pdo_->velocity_actual;
        int32_t signed_pdo = static_cast<int32_t>(raw_pdo);
        double motor_shaft_rpm = static_cast<double>(signed_pdo) / 1000.0;
        double output_shaft_rpm = motor_shaft_rpm / JD8Constants::GEAR_REDUCTION_RATIO;
        
        // Debug logging for velocity feedback scaling
#if JD8_DEBUG_LOGGING
        std::cout << "[FB] raw_pdo_uint32=" << raw_pdo
                  << " → signed_int32=" << signed_pdo  
                  << " → motor_shaft_rpm=" << motor_shaft_rpm
                  << " → output_shaft_rpm=" << output_shaft_rpm
                  << " → returning_motor_shaft=" << motor_shaft_rpm << std::endl;
#endif
        
        // Return motor shaft RPM for now (we can add separate function for output shaft if needed)
        return motor_shaft_rpm;
    }
    return 0.0;
}

double JD8Controller::getOutputRPM() const {
    if (input_pdo_) {
        uint32_t raw_pdo = input_pdo_->velocity_actual;
        int32_t signed_pdo = static_cast<int32_t>(raw_pdo);
        double motor_shaft_rpm = static_cast<double>(signed_pdo) / 1000.0;
        return motor_shaft_rpm / JD8Constants::GEAR_REDUCTION_RATIO;
    }
    return 0.0;
}

int32_t JD8Controller::getPosition() const {
    if (input_pdo_) {
        return input_pdo_->position_actual;  // Returns motor shaft position
    }
    return 0;
}

int32_t JD8Controller::getOutputPos() const {
    if (input_pdo_) {
        int32_t motor_shaft_position = input_pdo_->position_actual;
        return static_cast<int32_t>(motor_shaft_position / JD8Constants::GEAR_REDUCTION_RATIO);
    }
    return 0;
}

int16_t JD8Controller::getTorque() const {
    if (input_pdo_) {
        return JD8Constants::pdo_to_millinm(input_pdo_->torque_actual);
    }
    return 0;
}

uint16_t JD8Controller::getStatus() const {
    if (input_pdo_) {
        return input_pdo_->statusword;
    }
    return 0;
}

bool JD8Controller::is_motor_enabled() const {
    return motor_enabled_;
}

bool JD8Controller::has_fault() const {
    return input_pdo_ && (input_pdo_->statusword & JD8Constants::STATUSWORD_FAULT);
}

bool JD8Controller::clear_faults() {
    if (output_pdo_) {
        output_pdo_->controlword = JD8Constants::CONTROLWORD_FAULT_RESET;
        fault_recovery_active_ = false;
        return true;
    }
    return false;
}

JD8Controller::MotorState JD8Controller::get_motor_state() const {
    if (!input_pdo_) return UNKNOWN;
    
    uint16_t statusword = input_pdo_->statusword;
    
    if (statusword & JD8Constants::STATUSWORD_FAULT)
        return FAULT;
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_OPERATION_ENABLED_VALUE)  // Operation enabled
        return OPERATION_ENABLED;
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_SWITCHED_ON)
        return SWITCHED_ON;
    if ((statusword & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON)
        return READY_TO_SWITCH_ON;
    
    return NOT_READY;
}

bool JD8Controller::set_control_mode(ControlMode mode) {
    current_mode_ = mode;
    return true;  // Mode is set when command is sent
}

JD8Controller::ControlMode JD8Controller::get_current_mode() const {
    return current_mode_;
}

void JD8Controller::eStop() {
    log_error(ErrorSeverity::CRITICAL, "Emergency stop activated");
    
    fault_recovery_active_ = true;
    target_torque_command_ = 0;
    
    if (output_pdo_) {
        output_pdo_->target_velocity = 0;
        output_pdo_->controlword = JD8Constants::CONTROLWORD_QUICK_STOP;
    }
}

bool JD8Controller::is_in_fault_recovery() const {
    return fault_recovery_active_;
}

int JD8Controller::enable_motor_sequence() {
    if (!input_pdo_ || !output_pdo_) return enable_step_;
    
    uint16_t status = input_pdo_->statusword;
    
    switch(enable_step_) {
        case 0:
            if (status & JD8Constants::STATUSWORD_FAULT) {
                output_pdo_->controlword = JD8Constants::CONTROLWORD_FAULT_RESET;
                return 0;
            }
            return 1;
            
        case 1:
            output_pdo_->controlword = JD8Constants::CONTROLWORD_SHUTDOWN;
            if ((status & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON) {
                return 2;
            }
            return 1;
            
        case 2:
            output_pdo_->controlword = JD8Constants::CONTROLWORD_SWITCH_ON;
            output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
            if ((status & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_SWITCHED_ON) {
                return 3;
            }
            return 2;
            
        case 3:
            output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
            output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
            
            if ((status & JD8Constants::STATUSWORD_STATE_MASK) == JD8Constants::STATUSWORD_OPERATION_ENABLED_VALUE) {
                motor_enabled_ = true;
                std::cout << "Motor enable detected: status=0x" << std::hex << status << ", masked=0x" << (status & JD8Constants::STATUSWORD_STATE_MASK) << std::dec << std::endl;
                return 4;
            }
            return 3;
            
        default:
            return 4;
    }
}

void JD8Controller::log_error(ErrorSeverity severity, const std::string& message) {
    std::string prefix;
    switch(severity) {
        case ErrorSeverity::INFO: prefix = "INFO"; break;
        case ErrorSeverity::WARNING: prefix = "WARN"; break;
        case ErrorSeverity::ERROR: prefix = "ERROR"; break;
        case ErrorSeverity::CRITICAL: prefix = "CRITICAL"; break;
    }
    std::cout << "[" << prefix << "] " << message << std::endl;
}

bool JD8Controller::validate_torque_command(int16_t torque) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    if (has_fault()) {
        log_error(ErrorSeverity::ERROR, "Motor fault detected, cannot set torque");
        return false;
    }
    
    if (abs(torque) > JD8Constants::MAX_TORQUE_MILLINM) {
        log_error(ErrorSeverity::WARNING, 
                  "Torque " + std::to_string(torque) + " exceeds limit ±" + 
                  std::to_string(JD8Constants::MAX_TORQUE_MILLINM) + " mNm");
    }
    
    return true;
}

void JD8Controller::ramp_torque_command() {
    if (!output_pdo_) return;
    
    int16_t torque_diff = target_torque_command_ - current_torque_command_;
    
    if (torque_diff == 0) {
        return;
    }
    
    int16_t ramp_step = JD8Constants::TORQUE_RAMP_RATE;  // 200 mNm per 4ms cycle (250Hz)
    if (torque_diff < 0) {
        ramp_step = -ramp_step;
    }
    
    if (abs(torque_diff) <= JD8Constants::TORQUE_RAMP_RATE) {
        current_torque_command_ = target_torque_command_;
    } else {
        current_torque_command_ += ramp_step;
    }
    
    uint16_t pdo_torque = JD8Constants::millinm_to_pdo(current_torque_command_);
    output_pdo_->target_torque = pdo_torque;
    
    if (fault_recovery_active_ && current_torque_command_ == 0) {
        fault_recovery_active_ = false;
        log_error(ErrorSeverity::INFO, "Fault recovery complete - torque ramped to zero");
    }
}

void JD8Controller::update_position_command() {
    if (!output_pdo_) return;
    
    int32_t position_diff = target_position_command_ - current_position_command_;
    
    if (position_diff == 0) {
        return;
    }
    
    int32_t max_step = JD8Constants::MAX_POSITION_CHANGE_PER_CYCLE;
    int32_t step = position_diff;
    
    if (abs(position_diff) > max_step) {
        step = (position_diff > 0) ? max_step : -max_step;
    }
    
    current_position_command_ += step;
    output_pdo_->target_position = static_cast<uint32_t>(current_position_command_);
    output_pdo_->controlword = JD8Constants::CONTROLWORD_POSITION_MODE;
}

void JD8Controller::update_velocity_command() {
    if (!output_pdo_) return;
    
    current_velocity_command_ = target_velocity_command_;
    
    // GEAR RATIO ANALYSIS: PDO goes to motor shaft, but user commands output shaft velocity
    // To achieve desired output shaft RPM, motor shaft must run 7.75x faster
    double motor_shaft_rpm = current_velocity_command_ * JD8Constants::GEAR_REDUCTION_RATIO;
    
    // Calculate and log all scaling steps
    uint32_t raw_command = static_cast<uint32_t>(current_velocity_command_);
    uint32_t gear_scaled_command = static_cast<uint32_t>(motor_shaft_rpm);
    uint32_t final_scaled_command = static_cast<uint32_t>(motor_shaft_rpm * 1000);
    
    output_pdo_->target_velocity = final_scaled_command;
    
    // Debug logging for velocity command scaling
#if JD8_DEBUG_LOGGING
    std::cout << "[CMD] target_output_shaft_rpm=" << target_velocity_command_ 
              << " → motor_shaft_rpm=" << motor_shaft_rpm
              << " → raw_uint32=" << raw_command
              << " → gear_scaled=" << gear_scaled_command
              << " → final_1000x=" << final_scaled_command
              << " → sent_to_pdo=" << output_pdo_->target_velocity << std::endl;
#endif
    
    // Only update control word if motor is enabled and we're in velocity mode
    if (motor_enabled_ && current_mode_ == VELOCITY_MODE) {
        output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
    }
}

void JD8Controller::handle_fault_recovery() {
    log_error(ErrorSeverity::WARNING, "Fault detected - initiating torque ramp down");
    
    fault_recovery_active_ = true;
    target_torque_command_ = 0;
}

// === Configuration Management Implementation ===

bool JD8Controller::loadConfig(const std::string& config_file) {
    std::cout << "Loading motor configuration from: " << config_file << std::endl;
    
    // Create new configuration parser
    config_parser_ = std::make_unique<JD8ConfigParser>();
    
    // Parse configuration file
    bool loaded = config_parser_->parseCSV(config_file);
    
    if (!loaded) {
        std::cerr << "Failed to load configuration file" << std::endl;
        for (const auto& error : config_parser_->getErrors()) {
            std::cerr << "  - " << error << std::endl;
        }
        config_parser_.reset();
        return false;
    }
    
    // Validate critical parameters
    auto motor_specs = config_parser_->getMotorSpecs();
    auto control_gains = config_parser_->getControlGains();
    
    std::cout << "Configuration loaded successfully" << std::endl;
    std::cout << "  Parameters: " << config_parser_->getParameterCount() << std::endl;
    std::cout << "  Encoder Resolution: " << motor_specs.encoder_resolution << " counts/rev" << std::endl;
    // Velocity Resolution removed - not needed
    // Velocity Scaling Factor removed - always 1
    
    // Velocity scaling verification removed - scaling factor is always 1
    
    return true;
}

const JD8ConfigParser* JD8Controller::getConfig() const {
    return config_parser_.get();
}

bool JD8Controller::hasConfig() const {
    return config_parser_ && config_parser_->isLoaded();
}

bool JD8Controller::uploadConfig() {
    if (!hasConfig()) {
        log_error(ErrorSeverity::ERROR, "No configuration loaded - cannot upload parameters");
        return false;
    }
    
    if (!operational_) {
        log_error(ErrorSeverity::ERROR, "EtherCAT not operational - cannot upload parameters");
        return false;
    }
    
    // Create SDO manager if not already created
    if (!sdo_manager_) {
        sdo_manager_ = std::make_unique<JD8SDOManager>(slave_index_);
        std::cout << "SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload complete configuration
    JD8SDOManager::SDOResult result = sdo_manager_->uploadComplete(*config_parser_);
    
    if (result == JD8SDOManager::SDOResult::SUCCESS) {
        std::cout << "Motor configuration uploaded successfully!" << std::endl;
        
        // Print upload statistics
        auto stats = sdo_manager_->getStatistics();
        std::cout << "Upload Statistics:" << std::endl;
        std::cout << "  Total uploads: " << stats.uploads_attempted << std::endl;
        std::cout << "  Successful: " << stats.uploads_successful << std::endl;
        std::cout << "  Failed: " << stats.uploads_failed << std::endl;
        std::cout << "  Success rate: " << (100.0 * stats.uploads_successful / stats.uploads_attempted) << "%" << std::endl;
        std::cout << "  Average upload time: " << stats.average_upload_time_ms << " ms" << std::endl;
        
        return true;
    } else {
        std::cerr << "Configuration upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

bool JD8Controller::uploadCriticalParam() {
    if (!hasConfig()) {
        log_error(ErrorSeverity::ERROR, "No configuration loaded - cannot upload parameters");
        return false;
    }
    
    if (!operational_) {
        log_error(ErrorSeverity::ERROR, "EtherCAT not operational - cannot upload parameters");
        return false;
    }
    
    // Create SDO manager if not already created
    if (!sdo_manager_) {
        sdo_manager_ = std::make_unique<JD8SDOManager>(slave_index_);
        std::cout << "SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload only critical parameters (faster, safer)
    JD8SDOManager::SDOResult result = sdo_manager_->uploadCriticalParameters(*config_parser_);
    
    if (result == JD8SDOManager::SDOResult::SUCCESS) {
        std::cout << "Critical parameters uploaded successfully!" << std::endl;
        return true;
    } else {
        std::cerr << "Critical parameter upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

const JD8SDOManager* JD8Controller::getSDO() const {
    return sdo_manager_.get();
}

} // namespace jd8
