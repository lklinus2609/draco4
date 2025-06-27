/**
 * @file motor_sdo_manager.hpp
 * @brief Motor SDO (Service Data Object) Manager
 * 
 * Handles EtherCAT SDO communication for uploading motor configuration
 * parameters, control gains, and safety limits to Synapticon servo drives.
 * Provides safe parameter upload with validation and verification.
 */

#pragma once

#include "motor_configuration.hpp"
#include <soem/ethercat.h>
#include <string>
#include <vector>
#include <cstdint>
#include "motor_pdo_structures.hpp"

namespace synapticon_motor {

/**
 * @class MotorSDOManager
 * @brief EtherCAT SDO communication manager for motor configuration
 * 
 * Provides safe and reliable SDO communication for uploading motor
 * parameters, control gains, and configuration data to Synapticon servo drives.
 * Includes parameter validation, error handling, and verification.
 */
class MotorSDOManager {
public:
    /**
     * @brief SDO operation result codes
     */
    enum class SDOResult {
        SUCCESS,           ///< Operation completed successfully
        TIMEOUT,           ///< Communication timeout
        INVALID_PARAMETER, ///< Parameter validation failed
        COMMUNICATION_ERROR, ///< EtherCAT communication error
        VERIFICATION_FAILED, ///< Parameter verification failed
        SLAVE_NOT_FOUND    ///< Target slave not found
    };
    
    /**
     * @brief SDO parameter data types
     */
    enum class SDODataType {
        UINT8 = 1,    ///< 8-bit unsigned integer
        UINT16 = 2,   ///< 16-bit unsigned integer
        UINT32 = 4,   ///< 32-bit unsigned integer
        INT32 = 4,    ///< 32-bit signed integer
        FLOAT32 = 4,  ///< 32-bit floating point
        FLOAT64 = 8   ///< 64-bit floating point
    };
    
    /**
     * @brief Constructor
     * @param slave_index EtherCAT slave index
     */
    MotorSDOManager(int slave_index = 1);
    
    /**
     * @brief Destructor
     */
    ~MotorSDOManager();
    
    // === Core SDO Operations ===
    
    /**
     * @brief Upload 32-bit parameter to motor drive
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param value Parameter value to upload
     * @param data_type Data type specification
     * @return SDO operation result
     */
    SDOResult upload32(uint16_t index, uint8_t subindex, uint32_t value, SDODataType data_type = SDODataType::UINT32);
    
    /**
     * @brief Upload floating-point parameter to motor drive
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param value Parameter value to upload
     * @return SDO operation result
     */
    SDOResult uploadFloat(uint16_t index, uint8_t subindex, double value);
    
    /**
     * @brief Download parameter from motor drive
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param value Reference to store downloaded value
     * @param data_type Data type specification
     * @return SDO operation result
     */
    SDOResult download32(uint16_t index, uint8_t subindex, uint32_t& value, SDODataType data_type = SDODataType::UINT32);
    
    /**
     * @brief Download floating-point parameter from motor drive
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param value Reference to store downloaded value
     * @return SDO operation result
     */
    SDOResult downloadFloat(uint16_t index, uint8_t subindex, double& value);
    
    /**
     * @brief Verify uploaded parameter by reading it back
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param expected_value Expected parameter value
     * @param tolerance Tolerance for floating-point comparison
     * @return true if verification successful
     */
    bool verify32(uint16_t index, uint8_t subindex, uint32_t expected_value);
    bool verifyFloat(uint16_t index, uint8_t subindex, double expected_value, double tolerance = 0.001);
    
    // === High-Level Configuration Upload ===
    
    /**
     * @brief Upload all control gains from configuration
     * @param gains Control gains structure
     * @return SDO operation result
     */
    SDOResult uploadControlGains(const MotorConfigParser::ControlGains& gains);
    
    /**
     * @brief Upload motor specifications
     * @param specs Motor specifications structure
     * @return SDO operation result
     */
    SDOResult uploadMotorSpecs(const MotorConfigParser::MotorSpecs& specs);
    
    /**
     * @brief Upload safety limits
     * @param limits Safety limits structure
     * @return SDO operation result
     */
    SDOResult uploadSafetyLimits(const MotorConfigParser::SafetyLimits& limits);
    
    /**
     * @brief Upload complete configuration from parser
     * @param config Configuration parser with loaded parameters
     * @return SDO operation result
     */
    SDOResult uploadComplete(const MotorConfigParser& config);
    
    // === Batch Operations ===
    
    /**
     * @brief Upload critical parameters only (gains + limits)
     * @param config Configuration parser
     * @return SDO operation result
     */
    SDOResult uploadCriticalParameters(const MotorConfigParser& config);
    
    /**
     * @brief Upload parameters by priority (safety first, then performance)
     * @param config Configuration parser
     * @return SDO operation result
     */
    SDOResult uploadParamByPriority(const MotorConfigParser& config);
    
    // === Utility Functions ===
    
    /**
     * @brief Set SDO communication timeout
     * @param timeout_ms Timeout in milliseconds
     */
    void setTimeout(uint32_t timeout_ms);
    
    /**
     * @brief Enable/disable parameter verification after upload
     * @param enable true to enable verification
     */
    void setVerificationEnabled(bool enable);
    
    /**
     * @brief Get last error message
     * @return Last error message string
     */
    const std::string& getLastError() const;
    
    /**
     * @brief Get SDO operation statistics
     * @return Upload success rate and timing info
     */
    struct SDOStats {
        uint32_t uploads_attempted = 0;
        uint32_t uploads_successful = 0;
        uint32_t uploads_failed = 0;
        uint32_t verifications_attempted = 0;
        uint32_t verifications_successful = 0;
        double average_upload_time_ms = 0.0;
    };
    SDOStats getStatistics() const;
    
    /**
     * @brief Clear operation statistics
     */
    void clearStatistics();
    
    /**
     * @brief Convert SDO result to human-readable string
     * @param result SDO operation result
     * @return Result description string
     */
    static const char* resultToString(SDOResult result);

private:
    // === Internal Helper Functions ===
    
    /**
     * @brief Perform low-level SDO write operation
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param data Pointer to data buffer
     * @param data_size Size of data in bytes
     * @return SDO operation result
     */
    SDOResult performSDOWrite(uint16_t index, uint8_t subindex, const void* data, uint8_t data_size);
    
    /**
     * @brief Perform low-level SDO read operation
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param data Pointer to data buffer
     * @param data_size Size of data buffer
     * @return SDO operation result
     */
    SDOResult performSDORead(uint16_t index, uint8_t subindex, void* data, uint8_t data_size);
    
    /**
     * @brief Validate parameter before upload
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param value Parameter value
     * @return true if parameter is valid
     */
    bool validateParameter(uint16_t index, uint8_t subindex, double value);
    
    /**
     * @brief Log SDO operation with details
     * @param operation Operation description
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param result Operation result
     */
    void logSDOOperation(const std::string& operation, uint16_t index, uint8_t subindex, SDOResult result);
    
    /**
     * @brief Set last error message
     * @param error Error message
     */
    void setLastError(const std::string& error);
    
    /**
     * @brief Convert floating-point value to motor drive format
     * @param value Floating-point value
     * @return Motor drive formatted value
     */
    uint32_t floatToMotorFormat(double value);
    
    /**
     * @brief Convert motor drive format to floating-point
     * @param motor_value Motor drive formatted value
     * @return Floating-point value
     */
    double motorFormatToFloat(uint32_t motor_value);
    
    // === Member Variables ===
    
    int slave_index_;              ///< EtherCAT slave index
    uint32_t timeout_ms_;          ///< SDO communication timeout
    bool verification_enabled_;    ///< Parameter verification flag
    std::string last_error_;       ///< Last error message
    SDOStats statistics_;          ///< Operation statistics
    
    // === Parameter Validation Limits ===
    
    static constexpr double MAX_POSITION_GAIN_KP = 50000.0;    ///< Maximum position proportional gain
    static constexpr double MAX_POSITION_GAIN_KI = 100000.0;   ///< Maximum position integral gain
    static constexpr double MAX_VELOCITY_GAIN_KP = 10.0;       ///< Maximum velocity proportional gain
    static constexpr double MAX_VELOCITY_GAIN_KI = 100.0;      ///< Maximum velocity integral gain
    static constexpr double MAX_CURRENT_GAIN_KP = 1.0;         ///< Maximum current proportional gain
    static constexpr double MAX_CURRENT_GAIN_KI = 10.0;        ///< Maximum current integral gain
    
    // === Object Dictionary Index Constants ===
    // (Object dictionary indices moved to JD8Constants to avoid duplication)
};

} // namespace synapticon_motor