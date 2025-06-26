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
        std::cout << "✓ EtherCAT master initialized on " << interface_name_ << std::endl;
        initialized_ = true;
        return true;
    } else {
        std::cerr << "✗ Failed to initialize EtherCAT master on " << interface_name_ << std::endl;
        std::cerr << "  Try running with sudo" << std::endl;
        return false;
    }
}

bool JD8Controller::scan_network() {
    if (!initialized_) {
        std::cerr << "✗ Master not initialized" << std::endl;
        return false;
    }
    
    std::cout << "Starting network scan..." << std::endl;
    
    int slave_count = ec_config_init(FALSE);
    
    if (slave_count > 0) {
        std::cout << "✓ Found " << ec_slavecount << " EtherCAT slaves:" << std::endl;
        
        for (int i = 1; i <= ec_slavecount; i++) {
            std::cout << "  Slave " << i << ": " << ec_slave[i].name 
                      << " (Product: 0x" << std::hex << ec_slave[i].eep_id << std::dec << ")" << std::endl;
        }
        return true;
    } else {
        std::cout << "✗ No slaves found on network" << std::endl;
        return false;
    }
}

bool JD8Controller::configure_slaves() {
    if (ec_slavecount == 0) {
        std::cerr << "✗ No slaves to configure" << std::endl;
        return false;
    }
    
    ec_config_map(&IOmap_);
    ec_configdc();
    
    std::cout << "✓ Process data mapped - Output bytes: " << ec_slave[slave_index_].Obytes 
              << ", Input bytes: " << ec_slave[slave_index_].Ibytes << std::endl;
    
    if (slave_index_ <= ec_slavecount) {
        output_pdo_ = reinterpret_cast<OutputPDO*>(ec_slave[slave_index_].outputs);
        input_pdo_ = reinterpret_cast<InputPDO*>(ec_slave[slave_index_].inputs);
        
        std::memset(output_pdo_, 0, sizeof(OutputPDO));
        
        std::cout << "✓ PDO mapping configured for slave " << slave_index_ << std::endl;
        return true;
    } else {
        std::cerr << "✗ Invalid slave index: " << slave_index_ << std::endl;
        return false;
    }
}

bool JD8Controller::start_operation() {
    std::cout << "Slaves mapped, state to SAFE_OP" << std::endl;
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        std::cerr << "✗ Not all slaves reached SAFE_OP state" << std::endl;
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
        std::cout << "✓ Operational state reached for all slaves" << std::endl;
        operational_ = true;
        return true;
    } else {
        std::cerr << "✗ Not all slaves reached operational state" << std::endl;
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
        std::cout << "✓ EtherCAT master shut down" << std::endl;
    }
}

void JD8Controller::update() {
    if (operational_) {
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        if (current_mode_ == TORQUE_MODE || fault_recovery_active_) {
            ramp_torque_command();
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
                      << ", State=" << get_motor_state_string() << std::dec << std::endl;
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
    
    if (output_pdo_) {
        output_pdo_->controlword = JD8Constants::CONTROLWORD_SHUTDOWN;
        output_pdo_->target_velocity = 0;
        output_pdo_->target_torque = 0;
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
    
    if (output_pdo_) {
        output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
        output_pdo_->target_velocity = JD8Constants::rpm_to_counts(rpm);
        output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
        current_mode_ = VELOCITY_MODE;
        return true;
    }
    
    return false;
}

bool JD8Controller::set_position_counts(int32_t position) {
    if (!motor_enabled_) {
        log_error(ErrorSeverity::ERROR, "Motor not enabled");
        return false;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_position_time_);
    
    if (elapsed.count() > 0 && last_position_time_ != std::chrono::steady_clock::time_point{}) {
        int32_t current_pos = get_actual_position_counts();
        int32_t position_change = abs(position - current_pos);
        int32_t max_change = JD8Constants::MAX_POSITION_CHANGE_PER_CYCLE * elapsed.count();  // Rate limit based on 4ms cycles (250Hz)
        
        if (position_change > max_change) {
            log_error(ErrorSeverity::WARNING, 
                      "Position change " + std::to_string(position_change) + 
                      " counts exceeds rate limit of " + std::to_string(max_change) + 
                      " counts. Command will be applied gradually.");
            
            int32_t direction = (position > current_pos) ? 1 : -1;
            position = current_pos + (direction * max_change);
        }
    }
    
    if (output_pdo_) {
        output_pdo_->modes_of_operation = JD8Constants::POSITION_MODE;
        output_pdo_->target_position = static_cast<uint32_t>(position);
        output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
        current_mode_ = POSITION_MODE;
        
        last_position_command_ = position;
        last_position_time_ = now;
        
        return true;
    }
    
    return false;
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
        output_pdo_->modes_of_operation = JD8Constants::TORQUE_MODE;
        output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
        current_mode_ = TORQUE_MODE;
    }
    
    return true;
}

const char* JD8Controller::get_motor_state_string() const {
    if (!input_pdo_) return "NO_DATA";
    
    uint16_t statusword = input_pdo_->statusword;
    
    if (statusword & JD8Constants::STATUSWORD_FAULT)
        return "FAULT";
    if ((statusword & 0x006F) == 0x0027)
        return "OPERATION ENABLED";
    if ((statusword & 0x006F) == JD8Constants::STATUSWORD_SWITCHED_ON)
        return "SWITCHED ON";
    if ((statusword & 0x006F) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON)
        return "READY TO SWITCH ON";
    if ((statusword & 0x004F) == 0x0040)
        return "SWITCH ON DISABLED";
    return "UNKNOWN";
}

int JD8Controller::get_actual_velocity_rpm() const {
    if (input_pdo_) {
        return JD8Constants::rpm_from_counts(input_pdo_->velocity_actual);
    }
    return 0;
}

double JD8Controller::get_actual_velocity_rpm_precise() const {
    if (input_pdo_) {
        return JD8Constants::rpm_from_counts_precise(input_pdo_->velocity_actual);
    }
    return 0.0;
}

int32_t JD8Controller::get_actual_position_counts() const {
    if (input_pdo_) {
        return input_pdo_->position_actual;
    }
    return 0;
}

int16_t JD8Controller::get_actual_torque_millinm() const {
    if (input_pdo_) {
        return JD8Constants::pdo_to_millinm(input_pdo_->torque_actual);
    }
    return 0;
}

uint16_t JD8Controller::get_status_word() const {
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
    if ((statusword & 0x006F) == 0x0027)  // Operation enabled
        return OPERATION_ENABLED;
    if ((statusword & 0x006F) == JD8Constants::STATUSWORD_SWITCHED_ON)
        return SWITCHED_ON;
    if ((statusword & 0x006F) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON)
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

void JD8Controller::emergency_stop() {
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
            if ((status & 0x006F) == JD8Constants::STATUSWORD_READY_TO_SWITCH_ON) {
                return 2;
            }
            return 1;
            
        case 2:
            output_pdo_->controlword = JD8Constants::CONTROLWORD_SWITCH_ON;
            output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
            if ((status & 0x006F) == JD8Constants::STATUSWORD_SWITCHED_ON) {
                return 3;
            }
            return 2;
            
        case 3:
            output_pdo_->controlword = JD8Constants::CONTROLWORD_ENABLE_OPERATION;
            output_pdo_->modes_of_operation = JD8Constants::VELOCITY_MODE;
            
            if ((status & 0x006F) == 0x0027) {
                motor_enabled_ = true;
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

void JD8Controller::handle_fault_recovery() {
    log_error(ErrorSeverity::WARNING, "Fault detected - initiating torque ramp down");
    
    fault_recovery_active_ = true;
    target_torque_command_ = 0;
}

// === Configuration Management Implementation ===

bool JD8Controller::load_configuration(const std::string& config_file) {
    std::cout << "Loading motor configuration from: " << config_file << std::endl;
    
    // Create new configuration parser
    config_parser_ = std::make_unique<JD8ConfigParser>();
    
    // Parse configuration file
    bool loaded = config_parser_->parseCSV(config_file);
    
    if (!loaded) {
        std::cerr << "✗ Failed to load configuration file" << std::endl;
        for (const auto& error : config_parser_->getErrors()) {
            std::cerr << "  - " << error << std::endl;
        }
        config_parser_.reset();
        return false;
    }
    
    // Validate critical parameters
    auto motor_specs = config_parser_->getMotorSpecs();
    auto control_gains = config_parser_->getControlGains();
    
    std::cout << "✓ Configuration loaded successfully" << std::endl;
    std::cout << "  Parameters: " << config_parser_->getParameterCount() << std::endl;
    std::cout << "  Encoder Resolution: " << motor_specs.encoder_resolution << " counts/rev" << std::endl;
    std::cout << "  Velocity Resolution: " << motor_specs.velocity_resolution << std::endl;
    std::cout << "  Velocity Scaling Factor: " << config_parser_->calculateVelocityScalingFactor() << std::endl;
    
    // Verify scaling factor matches our corrected constants
    double config_scaling = config_parser_->calculateVelocityScalingFactor();
    double code_scaling = JD8Constants::VELOCITY_FACTOR;
    
    if (std::abs(config_scaling - code_scaling) > 0.001) {
        log_error(ErrorSeverity::WARNING, 
                  "Velocity scaling mismatch: Config=" + std::to_string(config_scaling) + 
                  ", Code=" + std::to_string(code_scaling));
    } else {
        std::cout << "✓ Velocity scaling factor verified: " << code_scaling << std::endl;
    }
    
    return true;
}

const JD8ConfigParser* JD8Controller::get_configuration() const {
    return config_parser_.get();
}

bool JD8Controller::has_configuration() const {
    return config_parser_ && config_parser_->isLoaded();
}

bool JD8Controller::upload_configuration() {
    if (!has_configuration()) {
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
        std::cout << "✓ SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload complete configuration
    JD8SDOManager::SDOResult result = sdo_manager_->uploadCompleteConfiguration(*config_parser_);
    
    if (result == JD8SDOManager::SDOResult::SUCCESS) {
        std::cout << "✓ Motor configuration uploaded successfully!" << std::endl;
        
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
        std::cerr << "✗ Configuration upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

bool JD8Controller::upload_critical_parameters() {
    if (!has_configuration()) {
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
        std::cout << "✓ SDO manager created for slave " << slave_index_ << std::endl;
    }
    
    // Upload only critical parameters (faster, safer)
    JD8SDOManager::SDOResult result = sdo_manager_->uploadCriticalParameters(*config_parser_);
    
    if (result == JD8SDOManager::SDOResult::SUCCESS) {
        std::cout << "✓ Critical parameters uploaded successfully!" << std::endl;
        return true;
    } else {
        std::cerr << "✗ Critical parameter upload failed: " << sdo_manager_->getLastError() << std::endl;
        return false;
    }
}

const JD8SDOManager* JD8Controller::get_sdo_manager() const {
    return sdo_manager_.get();
}

} // namespace jd8
