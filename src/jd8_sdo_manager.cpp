/**
 * @file jd8_sdo_manager.cpp
 * @brief Implementation of JD8SDOManager class
 * 
 * Provides EtherCAT SDO communication for uploading motor configuration
 * parameters, control gains, and safety limits to JD8 servo drives.
 */

#include "jd8_sdo_manager.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <cmath>

namespace jd8 {

JD8SDOManager::JD8SDOManager(int slave_index) 
    : slave_index_(slave_index), timeout_ms_(1000), verification_enabled_(true) {
    clearStatistics();
}

JD8SDOManager::~JD8SDOManager() {
}

// === Core SDO Operations ===

JD8SDOManager::SDOResult JD8SDOManager::uploadParameter32(uint16_t index, uint8_t subindex, uint32_t value, SDODataType data_type) {
    auto start_time = std::chrono::steady_clock::now();
    
    statistics_.uploads_attempted++;
    
    // Validate parameter before upload
    if (!validateParameter(index, subindex, static_cast<double>(value))) {
        setLastError("Parameter validation failed for 0x" + 
                    std::to_string(index) + "," + std::to_string(subindex));
        statistics_.uploads_failed++;
        return SDOResult::INVALID_PARAMETER;
    }
    
    // Determine data size from type
    uint8_t data_size = static_cast<uint8_t>(data_type);
    
    // Perform SDO write
    SDOResult result = performSDOWrite(index, subindex, &value, data_size);
    
    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double upload_time = duration.count() / 1000.0; // Convert to milliseconds
    
    if (result == SDOResult::SUCCESS) {
        statistics_.uploads_successful++;
        
        // Update average timing
        double total_time = statistics_.average_upload_time_ms * (statistics_.uploads_successful - 1);
        statistics_.average_upload_time_ms = (total_time + upload_time) / statistics_.uploads_successful;
        
        // Verify parameter if enabled
        if (verification_enabled_) {
            statistics_.verifications_attempted++;
            if (verifyParameter32(index, subindex, value)) {
                statistics_.verifications_successful++;
            } else {
                result = SDOResult::VERIFICATION_FAILED;
            }
        }
    } else {
        statistics_.uploads_failed++;
    }
    
    logSDOOperation("UPLOAD_UINT32", index, subindex, result);
    return result;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadParameterFloat(uint16_t index, uint8_t subindex, double value) {
    auto start_time = std::chrono::steady_clock::now();
    
    statistics_.uploads_attempted++;
    
    // Validate parameter before upload
    if (!validateParameter(index, subindex, value)) {
        setLastError("Parameter validation failed for 0x" + 
                    std::to_string(index) + "," + std::to_string(subindex));
        statistics_.uploads_failed++;
        return SDOResult::INVALID_PARAMETER;
    }
    
    // Convert float to motor format
    uint32_t motor_value = floatToMotorFormat(value);
    
    // Perform SDO write
    SDOResult result = performSDOWrite(index, subindex, &motor_value, 4);
    
    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double upload_time = duration.count() / 1000.0;
    
    if (result == SDOResult::SUCCESS) {
        statistics_.uploads_successful++;
        
        // Update average timing
        double total_time = statistics_.average_upload_time_ms * (statistics_.uploads_successful - 1);
        statistics_.average_upload_time_ms = (total_time + upload_time) / statistics_.uploads_successful;
        
        // Verify parameter if enabled
        if (verification_enabled_) {
            statistics_.verifications_attempted++;
            if (verifyParameterFloat(index, subindex, value)) {
                statistics_.verifications_successful++;
            } else {
                result = SDOResult::VERIFICATION_FAILED;
            }
        }
    } else {
        statistics_.uploads_failed++;
    }
    
    logSDOOperation("UPLOAD_FLOAT", index, subindex, result);
    return result;
}

JD8SDOManager::SDOResult JD8SDOManager::downloadParameter32(uint16_t index, uint8_t subindex, uint32_t& value, SDODataType data_type) {
    uint8_t data_size = static_cast<uint8_t>(data_type);
    SDOResult result = performSDORead(index, subindex, &value, data_size);
    
    logSDOOperation("DOWNLOAD_UINT32", index, subindex, result);
    return result;
}

JD8SDOManager::SDOResult JD8SDOManager::downloadParameterFloat(uint16_t index, uint8_t subindex, double& value) {
    uint32_t motor_value;
    SDOResult result = performSDORead(index, subindex, &motor_value, 4);
    
    if (result == SDOResult::SUCCESS) {
        value = motorFormatToFloat(motor_value);
    }
    
    logSDOOperation("DOWNLOAD_FLOAT", index, subindex, result);
    return result;
}

bool JD8SDOManager::verifyParameter32(uint16_t index, uint8_t subindex, uint32_t expected_value) {
    uint32_t read_value;
    SDOResult result = downloadParameter32(index, subindex, read_value);
    
    if (result != SDOResult::SUCCESS) {
        setLastError("Verification failed: Could not read parameter 0x" + 
                    std::to_string(index) + "," + std::to_string(subindex));
        return false;
    }
    
    if (read_value != expected_value) {
        setLastError("Verification failed: Expected " + std::to_string(expected_value) + 
                    ", got " + std::to_string(read_value));
        return false;
    }
    
    return true;
}

bool JD8SDOManager::verifyParameterFloat(uint16_t index, uint8_t subindex, double expected_value, double tolerance) {
    double read_value;
    SDOResult result = downloadParameterFloat(index, subindex, read_value);
    
    if (result != SDOResult::SUCCESS) {
        setLastError("Verification failed: Could not read parameter 0x" + 
                    std::to_string(index) + "," + std::to_string(subindex));
        return false;
    }
    
    if (std::abs(read_value - expected_value) > tolerance) {
        setLastError("Verification failed: Expected " + std::to_string(expected_value) + 
                    ", got " + std::to_string(read_value) + " (tolerance=" + std::to_string(tolerance) + ")");
        return false;
    }
    
    return true;
}

// === High-Level Configuration Upload ===

JD8SDOManager::SDOResult JD8SDOManager::uploadControlGains(const JD8ConfigParser::ControlGains& gains) {
    std::cout << "Uploading control gains..." << std::endl;
    
    SDOResult result;
    
    // Upload position gains (0x2010)
    std::cout << "  Position gains: KP=" << gains.position_kp << ", KI=" << gains.position_ki << ", KD=" << gains.position_kd << std::endl;
    
    result = uploadParameterFloat(JD8Constants::POSITION_GAINS_INDEX, 1, gains.position_kp);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload position KP gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::POSITION_GAINS_INDEX, 2, gains.position_ki);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload position KI gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::POSITION_GAINS_INDEX, 3, gains.position_kd);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload position KD gain");
        return result;
    }
    
    // Upload velocity gains (0x2011)
    std::cout << "  Velocity gains: KP=" << gains.velocity_kp << ", KI=" << gains.velocity_ki << ", KD=" << gains.velocity_kd << std::endl;
    
    result = uploadParameterFloat(JD8Constants::VELOCITY_GAINS_INDEX, 1, gains.velocity_kp);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload velocity KP gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::VELOCITY_GAINS_INDEX, 2, gains.velocity_ki);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload velocity KI gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::VELOCITY_GAINS_INDEX, 3, gains.velocity_kd);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload velocity KD gain");
        return result;
    }
    
    // Upload current gains (0x2012)
    std::cout << "  Current gains: KP=" << gains.current_kp << ", KI=" << gains.current_ki << ", KD=" << gains.current_kd << std::endl;
    
    result = uploadParameterFloat(JD8Constants::CURRENT_GAINS_INDEX, 1, gains.current_kp);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload current KP gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::CURRENT_GAINS_INDEX, 2, gains.current_ki);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload current KI gain");
        return result;
    }
    
    result = uploadParameterFloat(JD8Constants::CURRENT_GAINS_INDEX, 3, gains.current_kd);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload current KD gain");
        return result;
    }
    
    std::cout << "Control gains uploaded successfully" << std::endl;
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadMotorSpecs(const JD8ConfigParser::MotorSpecs& specs) {
    std::cout << "Uploading motor specifications..." << std::endl;
    
    SDOResult result;
    
    // Upload encoder resolution (0x2110,3)
    std::cout << "  Encoder resolution: " << specs.encoder_resolution << " counts/rev" << std::endl;
    result = uploadParameter32(JD8Constants::MOTOR_CONFIG_INDEX, 3, specs.encoder_resolution);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload encoder resolution");
        return result;
    }
    
    // Upload velocity resolution (0x6081,0)
    std::cout << "  Velocity resolution: " << specs.velocity_resolution << std::endl;
    result = uploadParameter32(JD8Constants::CIA402_PROFILE_VELOCITY_INDEX, 0, specs.velocity_resolution);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload velocity resolution");
        return result;
    }
    
    // Upload max speed (0x6080,0)
    std::cout << "  Max speed: " << specs.max_speed << std::endl;
    result = uploadParameter32(JD8Constants::CIA402_MAX_SPEED_INDEX, 0, specs.max_speed);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload max speed");
        return result;
    }
    
    std::cout << "Motor specifications uploaded successfully" << std::endl;
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadSafetyLimits(const JD8ConfigParser::SafetyLimits& limits) {
    std::cout << "Uploading safety limits..." << std::endl;
    
    SDOResult result;
    
    // Upload position limits (0x607D)
    std::cout << "  Position limits: " << limits.position_limit_min << " to " << limits.position_limit_max << std::endl;
    
    result = uploadParameter32(JD8Constants::CIA402_POSITION_LIMITS_INDEX, 1, static_cast<uint32_t>(limits.position_limit_min), SDODataType::INT32);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload minimum position limit");
        return result;
    }
    
    result = uploadParameter32(JD8Constants::CIA402_POSITION_LIMITS_INDEX, 2, static_cast<uint32_t>(limits.position_limit_max), SDODataType::INT32);
    if (result != SDOResult::SUCCESS) {
        setLastError("Failed to upload maximum position limit");
        return result;
    }
    
    std::cout << "Safety limits uploaded successfully" << std::endl;
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadCompleteConfiguration(const JD8ConfigParser& config) {
    std::cout << "\n=== Uploading Complete Configuration ===" << std::endl;
    
    SDOResult result;
    
    // 1. Upload safety limits first (highest priority)
    auto safety_limits = config.getSafetyLimits();
    result = uploadSafetyLimits(safety_limits);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "Failed to upload safety limits: " << getLastError() << std::endl;
        return result;
    }
    
    // 2. Upload motor specifications
    auto motor_specs = config.getMotorSpecs();
    result = uploadMotorSpecs(motor_specs);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "Failed to upload motor specifications: " << getLastError() << std::endl;
        return result;
    }
    
    // 3. Upload control gains (performance tuning)
    auto control_gains = config.getControlGains();
    result = uploadControlGains(control_gains);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "Failed to upload control gains: " << getLastError() << std::endl;
        return result;
    }
    
    std::cout << "\nComplete configuration uploaded successfully!" << std::endl;
    std::cout << "Motor is now configured with your tuned parameters." << std::endl;
    
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadCriticalParameters(const JD8ConfigParser& config) {
    std::cout << "\n=== Uploading Critical Parameters ===" << std::endl;
    
    SDOResult result;
    
    // Upload only the most critical parameters for basic operation
    auto control_gains = config.getControlGains();
    auto safety_limits = config.getSafetyLimits();
    
    // Safety first
    result = uploadSafetyLimits(safety_limits);
    if (result != SDOResult::SUCCESS) {
        return result;
    }
    
    // Core control gains for stable operation
    result = uploadControlGains(control_gains);
    if (result != SDOResult::SUCCESS) {
        return result;
    }
    
    std::cout << "Critical parameters uploaded successfully" << std::endl;
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::uploadParametersByPriority(const JD8ConfigParser& config) {
    std::cout << "\n=== Uploading Parameters by Priority ===" << std::endl;
    
    // Priority 1: Safety limits (must succeed)
    std::cout << "Priority 1: Safety limits..." << std::endl;
    auto safety_limits = config.getSafetyLimits();
    SDOResult result = uploadSafetyLimits(safety_limits);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "CRITICAL: Safety limits upload failed - aborting" << std::endl;
        return result;
    }
    
    // Priority 2: Control gains (important for performance)
    std::cout << "Priority 2: Control gains..." << std::endl;
    auto control_gains = config.getControlGains();
    result = uploadControlGains(control_gains);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "⚠ Warning: Control gains upload failed - motor may have poor performance" << std::endl;
        // Continue with lower priority parameters
    }
    
    // Priority 3: Motor specifications (nice to have)
    std::cout << "Priority 3: Motor specifications..." << std::endl;
    auto motor_specs = config.getMotorSpecs();
    result = uploadMotorSpecs(motor_specs);
    if (result != SDOResult::SUCCESS) {
        std::cerr << "⚠ Warning: Motor specifications upload failed" << std::endl;
        // Not critical for basic operation
    }
    
    std::cout << "Parameter upload by priority completed" << std::endl;
    return SDOResult::SUCCESS;
}

// === Utility Functions ===

void JD8SDOManager::setTimeout(uint32_t timeout_ms) {
    timeout_ms_ = timeout_ms;
}

void JD8SDOManager::setVerificationEnabled(bool enable) {
    verification_enabled_ = enable;
}

const std::string& JD8SDOManager::getLastError() const {
    return last_error_;
}

JD8SDOManager::SDOStats JD8SDOManager::getStatistics() const {
    return statistics_;
}

void JD8SDOManager::clearStatistics() {
    statistics_ = SDOStats{};
}

const char* JD8SDOManager::resultToString(SDOResult result) {
    switch (result) {
        case SDOResult::SUCCESS: return "SUCCESS";
        case SDOResult::TIMEOUT: return "TIMEOUT";
        case SDOResult::INVALID_PARAMETER: return "INVALID_PARAMETER";
        case SDOResult::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
        case SDOResult::VERIFICATION_FAILED: return "VERIFICATION_FAILED";
        case SDOResult::SLAVE_NOT_FOUND: return "SLAVE_NOT_FOUND";
        default: return "UNKNOWN";
    }
}

// === Private Helper Functions ===

JD8SDOManager::SDOResult JD8SDOManager::performSDOWrite(uint16_t index, uint8_t subindex, const void* data, uint8_t data_size) {
    // Check if slave exists
    if (slave_index_ > ec_slavecount || slave_index_ < 1) {
        setLastError("Invalid slave index: " + std::to_string(slave_index_));
        return SDOResult::SLAVE_NOT_FOUND;
    }
    
    // Perform SOEM SDO write
    int result = ec_SDOwrite(slave_index_, index, subindex, FALSE, data_size, const_cast<void*>(data), timeout_ms_ * 1000);
    
    if (result <= 0) {
        setLastError("SDO write failed for 0x" + std::to_string(index) + "," + std::to_string(subindex) + 
                    " (SOEM result: " + std::to_string(result) + ")");
        return (result == 0) ? SDOResult::TIMEOUT : SDOResult::COMMUNICATION_ERROR;
    }
    
    return SDOResult::SUCCESS;
}

JD8SDOManager::SDOResult JD8SDOManager::performSDORead(uint16_t index, uint8_t subindex, void* data, uint8_t data_size) {
    // Check if slave exists
    if (slave_index_ > ec_slavecount || slave_index_ < 1) {
        setLastError("Invalid slave index: " + std::to_string(slave_index_));
        return SDOResult::SLAVE_NOT_FOUND;
    }
    
    int psize = data_size;
    
    // Perform SOEM SDO read
    int result = ec_SDOread(slave_index_, index, subindex, FALSE, &psize, data, timeout_ms_ * 1000);
    
    if (result <= 0) {
        setLastError("SDO read failed for 0x" + std::to_string(index) + "," + std::to_string(subindex) + 
                    " (SOEM result: " + std::to_string(result) + ")");
        return (result == 0) ? SDOResult::TIMEOUT : SDOResult::COMMUNICATION_ERROR;
    }
    
    if (psize != data_size) {
        setLastError("SDO read size mismatch: expected " + std::to_string(data_size) + 
                    ", got " + std::to_string(psize));
        return SDOResult::COMMUNICATION_ERROR;
    }
    
    return SDOResult::SUCCESS;
}

bool JD8SDOManager::validateParameter(uint16_t index, uint8_t subindex, double value) {
    // Validate gain parameters to prevent dangerous values
    if (index == JD8Constants::POSITION_GAINS_INDEX) {
        if (subindex == 1 && (value < 0 || value > MAX_POSITION_GAIN_KP)) return false;
        if (subindex == 2 && (value < 0 || value > MAX_POSITION_GAIN_KI)) return false;
        if (subindex == 3 && (value < 0 || value > 1000.0)) return false; // KD limit
    }
    
    if (index == JD8Constants::VELOCITY_GAINS_INDEX) {
        if (subindex == 1 && (value < 0 || value > MAX_VELOCITY_GAIN_KP)) return false;
        if (subindex == 2 && (value < 0 || value > MAX_VELOCITY_GAIN_KI)) return false;
        if (subindex == 3 && (value < 0 || value > 10.0)) return false; // KD limit
    }
    
    if (index == JD8Constants::CURRENT_GAINS_INDEX) {
        if (subindex == 1 && (value < 0 || value > MAX_CURRENT_GAIN_KP)) return false;
        if (subindex == 2 && (value < 0 || value > MAX_CURRENT_GAIN_KI)) return false;
        if (subindex == 3 && (value < 0 || value > 1.0)) return false; // KD limit
    }
    
    // All other parameters pass validation
    return true;
}

void JD8SDOManager::logSDOOperation(const std::string& operation, uint16_t index, uint8_t subindex, SDOResult result) {
    std::cout << "[SDO] " << operation << " 0x" << std::hex << std::setfill('0') << std::setw(4) << index 
              << "," << std::dec << static_cast<int>(subindex) << " -> " << resultToString(result) << std::endl;
}

void JD8SDOManager::setLastError(const std::string& error) {
    last_error_ = error;
}

uint32_t JD8SDOManager::floatToMotorFormat(double value) {
    // Convert floating-point to motor drive's internal format
    // This may need adjustment based on specific motor drive documentation
    float float_val = static_cast<float>(value);
    uint32_t result;
    std::memcpy(&result, &float_val, sizeof(float));
    return result;
}

double JD8SDOManager::motorFormatToFloat(uint32_t motor_value) {
    // Convert motor drive's internal format to floating-point
    float result;
    std::memcpy(&result, &motor_value, sizeof(float));
    return static_cast<double>(result);
}

} // namespace jd8