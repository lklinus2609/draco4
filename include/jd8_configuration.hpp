/**
 * @file jd8_configuration.hpp
 * @brief JD8 Motor Configuration Parser
 * 
 * Parses JD8 configuration CSV files containing motor parameters, control gains,
 * and safety limits. Provides structured access to configuration data for
 * EtherCAT SDO parameter upload and motor setup.
 */

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <cstdint>
#include "jd8_pdo_structures.hpp"

namespace jd8 {

/**
 * @class JD8ConfigParser
 * @brief Configuration file parser for JD8 motor parameters
 * 
 * Parses CSV configuration files in the format: INDEX, SUBINDEX, VALUE
 * and provides structured access to motor parameters, control gains,
 * and safety limits.
 */
class JD8ConfigParser {
public:
    /**
     * @struct Parameter
     * @brief Single configuration parameter from CSV file
     */
    struct Parameter {
        uint16_t index;                                              ///< Object dictionary index
        uint8_t subindex;                                           ///< Object dictionary subindex
        std::variant<int32_t, uint32_t, double, std::string> value; ///< Parameter value
        std::string raw_value;                                      ///< Original string value from CSV
        
        Parameter() : index(0), subindex(0), value(0), raw_value("") {}
        Parameter(uint16_t idx, uint8_t sub, const std::string& val) 
            : index(idx), subindex(sub), raw_value(val) {
            parseValue(val);
        }
        
    private:
        void parseValue(const std::string& val);
    };
    
    /**
     * @struct ControlGains
     * @brief PID control gains for position, velocity, and current loops
     */
    struct ControlGains {
        // Position loop gains (0x2010)
        double position_kp = 0.0;    ///< Position proportional gain
        double position_ki = 0.0;    ///< Position integral gain
        double position_kd = 0.0;    ///< Position derivative gain
        
        // Velocity loop gains (0x2011)
        double velocity_kp = 0.0;    ///< Velocity proportional gain
        double velocity_ki = 0.0;    ///< Velocity integral gain
        double velocity_kd = 0.0;    ///< Velocity derivative gain
        
        // Current loop gains (0x2012)
        double current_kp = 0.0;     ///< Current proportional gain
        double current_ki = 0.0;     ///< Current integral gain
        double current_kd = 0.0;     ///< Current derivative gain
    };
    
    /**
     * @struct MotorSpecs
     * @brief Motor specifications and encoder configuration
     */
    struct MotorSpecs {
        uint32_t encoder_resolution = 524288;  ///< Encoder counts per revolution
        uint32_t velocity_resolution = 1;      ///< Velocity scaling factor
        uint32_t max_speed = 1000;             ///< Maximum motor speed
        uint16_t rated_current = 4000;         ///< Rated motor current
        uint16_t motor_poles = 4;              ///< Number of motor pole pairs
        uint16_t torque_constant = 1000;       ///< Motor torque constant
    };
    
    /**
     * @struct SafetyLimits
     * @brief Motor safety and operational limits
     */
    struct SafetyLimits {
        int32_t position_limit_min = -2147483648;  ///< Minimum position limit
        int32_t position_limit_max = 2147483647;   ///< Maximum position limit
        uint32_t following_error_window = 65535;  ///< Following error limit
        uint16_t max_torque = 3000;               ///< Maximum torque limit
    };
    
    /**
     * @brief Constructor
     */
    JD8ConfigParser();
    
    /**
     * @brief Destructor
     */
    ~JD8ConfigParser();
    
    // === Core Parsing Functions ===
    
    /**
     * @brief Parse configuration from CSV file
     * @param filename Path to CSV configuration file
     * @return true if parsing successful
     */
    bool parseCSV(const std::string& filename);
    
    /**
     * @brief Validate parsed parameters
     * @return true if all parameters are valid
     */
    bool validateParameters();
    
    /**
     * @brief Get number of parsed parameters
     * @return Number of parameters loaded
     */
    size_t getParameterCount() const;
    
    // === Parameter Access ===
    
    /**
     * @brief Get specific parameter by index and subindex
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @return Parameter if found, nullptr otherwise
     */
    const Parameter* getParameter(uint16_t index, uint8_t subindex) const;
    
    /**
     * @brief Get all parameters for a specific index
     * @param index Object dictionary index
     * @return Vector of parameters with matching index
     */
    std::vector<const Parameter*> getParametersByIndex(uint16_t index) const;
    
    // === Structured Data Access ===
    
    /**
     * @brief Extract control gains from configuration
     * @return ControlGains structure with PID parameters
     */
    ControlGains getControlGains() const;
    
    /**
     * @brief Extract motor specifications
     * @return MotorSpecs structure with motor parameters
     */
    MotorSpecs getMotorSpecs() const;
    
    /**
     * @brief Extract safety limits
     * @return SafetyLimits structure with operational limits
     */
    SafetyLimits getSafetyLimits() const;
    
    // === Critical RPM Parameters ===
    
    /**
     * @brief Get encoder resolution for RPM calculation
     * @return Encoder counts per revolution
     */
    uint32_t getEncoderResolution() const;
    
    /**
     * @brief Get velocity resolution factor
     * @return Velocity scaling factor for RPM calculation
     */
    uint32_t getVelocityResolution() const;
    
    /**
     * @brief Calculate proper velocity scaling factor
     * @return Calculated velocity factor for accurate RPM
     */
    double calculateVelocityScalingFactor() const;
    
    // === Utility Functions ===
    
    /**
     * @brief Check if configuration file was successfully loaded
     * @return true if configuration is loaded and valid
     */
    bool isLoaded() const;
    
    /**
     * @brief Get parsing error messages
     * @return Vector of error messages from parsing
     */
    const std::vector<std::string>& getErrors() const;
    
    /**
     * @brief Clear all loaded configuration data
     */
    void clear();
    
    /**
     * @brief Print configuration summary for debugging
     */
    void printSummary() const;

private:
    // === Internal Helper Functions ===
    
    /**
     * @brief Parse a single CSV line
     * @param line CSV line to parse
     * @param line_number Line number for error reporting
     * @return true if line parsed successfully
     */
    bool parseLine(const std::string& line, size_t line_number);
    
    /**
     * @brief Trim whitespace from string
     * @param str String to trim
     * @return Trimmed string
     */
    std::string trim(const std::string& str) const;
    
    /**
     * @brief Convert hex string to integer
     * @param hex_str Hexadecimal string (with or without 0x prefix)
     * @return Converted integer value
     */
    uint32_t parseHex(const std::string& hex_str) const;
    
    /**
     * @brief Add error message to error list
     * @param message Error message
     */
    void addError(const std::string& message);
    
    /**
     * @brief Get parameter value as specific type
     * @param param Parameter to convert
     * @return Converted value or default if conversion fails
     */
    template<typename T>
    T getParameterValue(const Parameter* param, T default_value = T{}) const;
    
    // === Member Variables ===
    
    std::unordered_map<uint32_t, Parameter> parameters_;  ///< Parsed parameters (key: index<<16 | subindex)
    std::vector<std::string> errors_;                     ///< Parsing error messages
    bool loaded_;                                         ///< Configuration loaded flag
    std::string filename_;                                ///< Source filename
    
    // === Parameter Index Constants ===
    // (Object dictionary indices moved to JD8Constants to avoid duplication)
};

} // namespace jd8