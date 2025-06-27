/**
 * @file jd8_configuration.cpp
 * @brief Implementation of JD8ConfigParser class
 * 
 * Provides CSV configuration file parsing for JD8 motor parameters,
 * control gains, and safety limits. Implements structured access to
 * configuration data for EtherCAT SDO parameter upload.
 */

#include "jd8_configuration.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cctype>

namespace jd8 {

// === Parameter Implementation ===

void JD8ConfigParser::Parameter::parseValue(const std::string& val) {
    std::string trimmed = val;
    // Remove leading/trailing whitespace
    trimmed.erase(trimmed.begin(), std::find_if(trimmed.begin(), trimmed.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    trimmed.erase(std::find_if(trimmed.rbegin(), trimmed.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), trimmed.end());
    
    if (trimmed.empty()) {
        value = 0;
        return;
    }
    
    // Try to parse as different types
    try {
        // Check if it's a string (contains non-numeric characters that aren't hex)
        if (trimmed.find_first_not_of("0123456789+-.") != std::string::npos &&
            trimmed.substr(0, 2) != "0x" && trimmed.substr(0, 2) != "0X") {
            // Contains letters or special chars - treat as string
            value = trimmed;
            return;
        }
        
        // Check for hexadecimal
        if (trimmed.substr(0, 2) == "0x" || trimmed.substr(0, 2) == "0X") {
            uint32_t hex_val = static_cast<uint32_t>(std::stoul(trimmed, nullptr, 16));
            value = hex_val;
            return;
        }
        
        // Check for decimal point - parse as double
        if (trimmed.find('.') != std::string::npos) {
            double double_val = std::stod(trimmed);
            value = double_val;
            return;
        }
        
        // Try to parse as signed integer first
        try {
            int32_t int_val = std::stoi(trimmed);
            value = int_val;
            return;
        } catch (const std::out_of_range&) {
            // Try unsigned if signed fails due to range
            uint32_t uint_val = static_cast<uint32_t>(std::stoul(trimmed));
            value = uint_val;
            return;
        }
        
    } catch (const std::exception&) {
        // If all parsing fails, store as string
        value = trimmed;
    }
}

// === JD8ConfigParser Implementation ===

JD8ConfigParser::JD8ConfigParser() : loaded_(false) {
}

JD8ConfigParser::~JD8ConfigParser() {
}

bool JD8ConfigParser::parseCSV(const std::string& filename) {
    clear();
    filename_ = filename;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        addError("Failed to open configuration file: " + filename);
        return false;
    }
    
    std::string line;
    size_t line_number = 0;
    size_t parsed_count = 0;
    
    std::cout << "Parsing configuration file: " << filename << std::endl;
    
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip empty lines and comments
        std::string trimmed_line = trim(line);
        if (trimmed_line.empty() || trimmed_line[0] == '#') {
            continue;
        }
        
        // Skip META line
        if (trimmed_line.find("META") == 0) {
            continue;
        }
        
        if (parseLine(trimmed_line, line_number)) {
            parsed_count++;
        }
    }
    
    file.close();
    
    if (parsed_count > 0) {
        loaded_ = true;
        std::cout << "Parsed " << parsed_count << " parameters from " << filename << std::endl;
        
        // Print summary of key parameters
        printSummary();
        
        return validateParameters();
    } else {
        addError("No valid parameters found in file");
        return false;
    }
}

bool JD8ConfigParser::parseLine(const std::string& line, size_t line_number) {
    std::stringstream ss(line);
    std::string index_str, subindex_str, value_str;
    
    // Parse CSV format: INDEX, SUBINDEX, VALUE
    if (!std::getline(ss, index_str, ',') ||
        !std::getline(ss, subindex_str, ',') ||
        !std::getline(ss, value_str)) {
        addError("Line " + std::to_string(line_number) + ": Invalid CSV format");
        return false;
    }
    
    try {
        // Parse index (hex format)
        uint16_t index = static_cast<uint16_t>(parseHex(trim(index_str)));
        
        // Parse subindex (decimal)
        uint8_t subindex = static_cast<uint8_t>(std::stoul(trim(subindex_str)));
        
        // Create parameter
        Parameter param(index, subindex, trim(value_str));
        
        // Store parameter (key: index<<16 | subindex)
        uint32_t key = (static_cast<uint32_t>(index) << 16) | subindex;
        parameters_[key] = param;
        
        return true;
        
    } catch (const std::exception& e) {
        addError("Line " + std::to_string(line_number) + ": Parsing error - " + e.what());
        return false;
    }
}

bool JD8ConfigParser::validateParameters() {
    if (parameters_.empty()) {
        addError("No parameters loaded");
        return false;
    }
    
    // Check for critical parameters
    bool has_encoder_res = getParameter(JD8Constants::MOTOR_CONFIG_INDEX, 3) != nullptr;
    bool has_velocity_res = getParameter(JD8Constants::CIA402_PROFILE_VELOCITY_INDEX, 0) != nullptr;
    
    if (!has_encoder_res) {
        addError("Missing critical parameter: Encoder resolution (0x2110,3)");
    }
    
    if (!has_velocity_res) {
        addError("Missing critical parameter: Velocity resolution (0x6081,0)");
    }
    
    return errors_.empty();
}

size_t JD8ConfigParser::getParameterCount() const {
    return parameters_.size();
}

const JD8ConfigParser::Parameter* JD8ConfigParser::getParameter(uint16_t index, uint8_t subindex) const {
    uint32_t key = (static_cast<uint32_t>(index) << 16) | subindex;
    auto it = parameters_.find(key);
    return (it != parameters_.end()) ? &it->second : nullptr;
}

std::vector<const JD8ConfigParser::Parameter*> JD8ConfigParser::getParametersByIndex(uint16_t index) const {
    std::vector<const Parameter*> result;
    
    for (const auto& pair : parameters_) {
        if ((pair.first >> 16) == index) {
            result.push_back(&pair.second);
        }
    }
    
    return result;
}

JD8ConfigParser::ControlGains JD8ConfigParser::getControlGains() const {
    ControlGains gains;
    
    // Position gains (0x2010)
    if (const Parameter* p = getParameter(JD8Constants::POSITION_GAINS_INDEX, 1)) {
        gains.position_kp = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::POSITION_GAINS_INDEX, 2)) {
        gains.position_ki = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::POSITION_GAINS_INDEX, 3)) {
        gains.position_kd = getParameterValue<double>(p, 0.0);
    }
    
    // Velocity gains (0x2011)
    if (const Parameter* p = getParameter(JD8Constants::VELOCITY_GAINS_INDEX, 1)) {
        gains.velocity_kp = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::VELOCITY_GAINS_INDEX, 2)) {
        gains.velocity_ki = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::VELOCITY_GAINS_INDEX, 3)) {
        gains.velocity_kd = getParameterValue<double>(p, 0.0);
    }
    
    // Current gains (0x2012)
    if (const Parameter* p = getParameter(JD8Constants::CURRENT_GAINS_INDEX, 1)) {
        gains.current_kp = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::CURRENT_GAINS_INDEX, 2)) {
        gains.current_ki = getParameterValue<double>(p, 0.0);
    }
    if (const Parameter* p = getParameter(JD8Constants::CURRENT_GAINS_INDEX, 3)) {
        gains.current_kd = getParameterValue<double>(p, 0.0);
    }
    
    return gains;
}

JD8ConfigParser::MotorSpecs JD8ConfigParser::getMotorSpecs() const {
    MotorSpecs specs;
    
    // Encoder resolution (0x2110,3)
    if (const Parameter* p = getParameter(JD8Constants::MOTOR_CONFIG_INDEX, 3)) {
        specs.encoder_resolution = getParameterValue<uint32_t>(p, 524288);
    }
    
    // Velocity resolution (0x6081,0)
    if (const Parameter* p = getParameter(JD8Constants::CIA402_PROFILE_VELOCITY_INDEX, 0)) {
        specs.velocity_resolution = getParameterValue<uint32_t>(p, 1);
    }
    
    // Max speed (0x6080,0)
    if (const Parameter* p = getParameter(JD8Constants::CIA402_MAX_SPEED_INDEX, 0)) {
        specs.max_speed = getParameterValue<uint32_t>(p, 1000);
    }
    
    // Rated current (0x2110,4)
    if (const Parameter* p = getParameter(JD8Constants::MOTOR_CONFIG_INDEX, 4)) {
        specs.rated_current = getParameterValue<uint16_t>(p, 4000);
    }
    
    return specs;
}

JD8ConfigParser::SafetyLimits JD8ConfigParser::getSafetyLimits() const {
    SafetyLimits limits;
    
    // Position limits (0x607D)
    if (const Parameter* p = getParameter(JD8Constants::CIA402_POSITION_LIMITS_INDEX, 1)) {
        limits.position_limit_min = getParameterValue<int32_t>(p, -2147483648);
    }
    if (const Parameter* p = getParameter(JD8Constants::CIA402_POSITION_LIMITS_INDEX, 2)) {
        limits.position_limit_max = getParameterValue<int32_t>(p, 2147483647);
    }
    
    return limits;
}

uint32_t JD8ConfigParser::getEncoderResolution() const {
    if (const Parameter* p = getParameter(JD8Constants::MOTOR_CONFIG_INDEX, 3)) {
        return getParameterValue<uint32_t>(p, 524288);
    }
    return 524288;  // Default fallback
}

uint32_t JD8ConfigParser::getVelocityResolution() const {
    if (const Parameter* p = getParameter(JD8Constants::CIA402_PROFILE_VELOCITY_INDEX, 0)) {
        return getParameterValue<uint32_t>(p, 1);
    }
    return 1;  // Default fallback
}

double JD8ConfigParser::calcVelScale() const {
    // Calculate proper velocity scaling based on encoder resolution and velocity resolution
    uint32_t encoder_res = getEncoderResolution();
    uint32_t velocity_res = getVelocityResolution();
    
    // Velocity scaling factor = velocity_resolution / encoder_resolution
    // This accounts for the drive's internal velocity units
    double scaling_factor = static_cast<double>(velocity_res) / static_cast<double>(encoder_res);
    
    return scaling_factor;
}

bool JD8ConfigParser::isLoaded() const {
    return loaded_;
}

const std::vector<std::string>& JD8ConfigParser::getErrors() const {
    return errors_;
}

void JD8ConfigParser::clear() {
    parameters_.clear();
    errors_.clear();
    loaded_ = false;
    filename_.clear();
}

void JD8ConfigParser::printSummary() const {
    if (!loaded_) {
        std::cout << "No configuration loaded" << std::endl;
        return;
    }
    
    std::cout << "\n=== Configuration Summary ===" << std::endl;
    
    // Motor specifications
    MotorSpecs specs = getMotorSpecs();
    std::cout << "Motor Specifications:" << std::endl;
    std::cout << "  Encoder Resolution: " << specs.encoder_resolution << " counts/rev" << std::endl;
    // Velocity Resolution removed - not needed
    std::cout << "  Max Speed: " << specs.max_speed << std::endl;
    std::cout << "  Rated Current: " << specs.rated_current << std::endl;
    
    // Control gains
    ControlGains gains = getControlGains();
    std::cout << "\nControl Gains:" << std::endl;
    std::cout << "  Position: KP=" << gains.position_kp << ", KI=" << gains.position_ki << ", KD=" << gains.position_kd << std::endl;
    std::cout << "  Velocity: KP=" << gains.velocity_kp << ", KI=" << gains.velocity_ki << ", KD=" << gains.velocity_kd << std::endl;
    std::cout << "  Current:  KP=" << gains.current_kp << ", KI=" << gains.current_ki << ", KD=" << gains.current_kd << std::endl;
    
    // Velocity scaling analysis removed - scaling is always 1
    
    std::cout << "\nTotal Parameters Loaded: " << parameters_.size() << std::endl;
    std::cout << "========================" << std::endl;
}

// === Private Helper Functions ===

std::string JD8ConfigParser::trim(const std::string& str) const {
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

uint32_t JD8ConfigParser::parseHex(const std::string& hex_str) const {
    std::string clean_hex = hex_str;
    
    // Remove 0x prefix if present
    if (clean_hex.size() > 2 && clean_hex.substr(0, 2) == "0x") {
        clean_hex = clean_hex.substr(2);
    }
    
    return static_cast<uint32_t>(std::stoul(clean_hex, nullptr, 16));
}

void JD8ConfigParser::addError(const std::string& message) {
    errors_.push_back(message);
    std::cerr << "[CONFIG ERROR] " << message << std::endl;
}

template<typename T>
T JD8ConfigParser::getParameterValue(const Parameter* param, T default_value) const {
    if (!param) {
        return default_value;
    }
    
    try {
        if constexpr (std::is_same_v<T, double>) {
            if (std::holds_alternative<double>(param->value)) {
                return std::get<double>(param->value);
            } else if (std::holds_alternative<int32_t>(param->value)) {
                return static_cast<double>(std::get<int32_t>(param->value));
            } else if (std::holds_alternative<uint32_t>(param->value)) {
                return static_cast<double>(std::get<uint32_t>(param->value));
            }
        } else if constexpr (std::is_same_v<T, uint32_t>) {
            if (std::holds_alternative<uint32_t>(param->value)) {
                return std::get<uint32_t>(param->value);
            } else if (std::holds_alternative<int32_t>(param->value)) {
                return static_cast<uint32_t>(std::get<int32_t>(param->value));
            }
        } else if constexpr (std::is_same_v<T, int32_t>) {
            if (std::holds_alternative<int32_t>(param->value)) {
                return std::get<int32_t>(param->value);
            } else if (std::holds_alternative<uint32_t>(param->value)) {
                return static_cast<int32_t>(std::get<uint32_t>(param->value));
            }
        } else if constexpr (std::is_same_v<T, uint16_t>) {
            if (std::holds_alternative<uint32_t>(param->value)) {
                return static_cast<uint16_t>(std::get<uint32_t>(param->value));
            } else if (std::holds_alternative<int32_t>(param->value)) {
                return static_cast<uint16_t>(std::get<int32_t>(param->value));
            }
        } else if constexpr (std::is_same_v<T, std::string>) {
            if (std::holds_alternative<std::string>(param->value)) {
                return std::get<std::string>(param->value);
            }
        }
    } catch (const std::exception&) {
        // Fall through to default
    }
    
    return default_value;
}

} // namespace jd8
