/**
 * @file jd8_pdo_structures.hpp
 * @brief JD8 EtherCAT PDO (Process Data Object) Structures
 * 
 * Defines the exact PDO mapping structures for JD8 servo motors as used
 * in EtherCAT communication. These structures must match the motor's
 * configured PDO mapping exactly for proper operation.
 */

#pragma once
#include <cstdint>

namespace jd8 {

/**
 * @struct OutputPDO
 * @brief Output PDO structure (Master to Slave)
 * 
 * Contains all output data sent from EtherCAT master to JD8 motor.
 * Structure size: 35 bytes, packed to match hardware layout.
 */
struct OutputPDO {
    uint16_t controlword;          ///< CIA402 control word [Bytes 0-1]
    uint8_t  modes_of_operation;   ///< Operating mode selection [Byte 2]
    uint16_t target_torque;        ///< Target torque value [Bytes 3-4]
    uint32_t target_position;      ///< Target position [Bytes 5-8]
    uint32_t target_velocity;      ///< Target velocity [Bytes 9-12]
    uint16_t torque_offset;        ///< Torque offset [Bytes 13-14]
    uint32_t tuning_command;       ///< Tuning parameters [Bytes 15-18]
    uint32_t physical_outputs;     ///< Digital outputs [Bytes 19-22]
    uint32_t bit_mask;            ///< Output bit mask [Bytes 23-26]
    uint32_t user_mosi;           ///< User-defined output [Bytes 27-30]
    uint32_t velocity_offset;     ///< Velocity offset [Bytes 31-34]
} __attribute__((packed));

/**
 * @struct InputPDO
 * @brief Input PDO structure (Slave to Master)
 * 
 * Contains all input data received from JD8 motor to EtherCAT master.
 * Structure size: 47 bytes, packed to match hardware layout.
 */
struct InputPDO {
    uint16_t statusword;                    ///< CIA402 status word [Bytes 0-1]
    uint8_t  modes_of_operation_display;    ///< Current operating mode [Byte 2]
    uint32_t position_actual;               ///< Actual position feedback [Bytes 3-6]
    uint32_t velocity_actual;               ///< Actual velocity feedback [Bytes 7-10]
    uint16_t torque_actual;                 ///< Actual torque feedback [Bytes 11-12]
    uint32_t position_following_error;      ///< Position following error [Bytes 13-16]
    uint32_t user_miso;                     ///< User-defined input [Bytes 17-20]
    uint32_t digital_inputs;                ///< Digital input states [Bytes 21-24]
    uint8_t  reserved[22];                  ///< Reserved padding [Bytes 25-46]
} __attribute__((packed));

/**
 * @class JD8Constants
 * @brief Constants and utility functions for JD8 motor control
 * 
 * Contains all motor-specific constants, conversion factors, and utility
 * functions needed for JD8 servo motor operation.
 */
class JD8Constants {
public:
    // === Motor Specifications ===
    static constexpr int COUNTS_PER_REV = 524288;         ///< Encoder counts per revolution
    static constexpr double GEAR_REDUCTION_RATIO = 7.75;  ///< Gear reduction ratio (motor:output = 7.75:1)
    static constexpr double VELOCITY_FACTOR = 0.147665;   ///< Velocity scaling factor (from config)
    static constexpr int MAX_VELOCITY_RPM = 1000;         ///< Maximum safe velocity in RPM
    
    // === Configuration-Based Parameters ===
    static constexpr uint32_t VELOCITY_RESOLUTION = 77419; ///< Drive velocity resolution from config
    static constexpr double CALCULATED_VELOCITY_FACTOR = static_cast<double>(VELOCITY_RESOLUTION) / static_cast<double>(COUNTS_PER_REV);
    
    // === Control Modes ===
    static constexpr uint8_t VELOCITY_MODE = 0x09;      ///< Profile velocity mode
    static constexpr uint8_t POSITION_MODE = 0x08;      ///< Profile position mode
    static constexpr uint8_t TORQUE_MODE = 0x0A;        ///< Torque control mode
    
    // === CIA402 Control Words ===
    static constexpr uint16_t CONTROLWORD_SHUTDOWN = 0x0006;           ///< Shutdown command
    static constexpr uint16_t CONTROLWORD_SWITCH_ON = 0x0007;          ///< Switch on command
    static constexpr uint16_t CONTROLWORD_ENABLE_OPERATION = 0x000F;   ///< Enable operation
    static constexpr uint16_t CONTROLWORD_DISABLE_VOLTAGE = 0x0000;    ///< Disable voltage
    static constexpr uint16_t CONTROLWORD_QUICK_STOP = 0x0002;         ///< Quick stop
    static constexpr uint16_t CONTROLWORD_FAULT_RESET = 0x0080;        ///< Fault reset
    
    // === CIA402 Status Word Masks ===
    static constexpr uint16_t STATUSWORD_READY_TO_SWITCH_ON = 0x0021;  ///< Ready to switch on
    static constexpr uint16_t STATUSWORD_SWITCHED_ON = 0x0023;         ///< Switched on
    static constexpr uint16_t STATUSWORD_OPERATION_ENABLED = 0x0237;   ///< Operation enabled
    static constexpr uint16_t STATUSWORD_FAULT = 0x0008;               ///< Fault detected
    
    // === Real-Time System Configuration ===
    static constexpr double RT_FREQUENCY_HZ = 250.0;         ///< RT kernel frequency: 250Hz
    static constexpr double CYCLE_TIME_MS = 4.0;             ///< RT cycle time: 4ms per cycle
    
    // === Torque Control Constants ===
    static constexpr int16_t RATED_TORQUE_MNM = 6000;        ///< Rated torque: 6 Nm = 6000 mNm
    static constexpr int16_t PDO_TORQUE_SCALE = 1000;        ///< PDO scale: 1000 = full torque
    static constexpr int16_t MAX_TORQUE_MILLINM = 3000;      ///< Max safe torque: 3 Nm
    static constexpr int16_t MIN_TORQUE_MILLINM = -3000;     ///< Min safe torque: -3 Nm
    static constexpr int16_t TORQUE_RAMP_RATE = 200;         ///< Torque ramp: 200 mNm per 4ms cycle (250Hz)
    
    // === Position Control Safety Limits ===
    static constexpr int32_t MAX_POSITION_CHANGE_PER_CYCLE = 200000;  ///< Max position change per 4ms cycle (250Hz)
    static constexpr int32_t MAX_POSITION_CHANGE_PER_SECOND = 50000000; ///< Max position change per second (250 cycles × 200k)
    
    // === Utility Functions ===
    
    /**
     * @brief Convert RPM to encoder counts for PDO commands
     * @param rpm Velocity in revolutions per minute
     * @return Velocity in encoder counts per minute (for PDO command)
     * @note Uses raw encoder counts - no scaling factor for commands
     */
    static int32_t rpm_to_counts(int rpm) {
        return static_cast<int32_t>((rpm * COUNTS_PER_REV) / 60.0);
    }
    
    /**
     * @brief Convert RPM to encoder counts with velocity scaling (for reference)
     * @param rpm Velocity in revolutions per minute
     * @return Velocity in scaled encoder counts per minute
     * @note This version applies velocity scaling factor - use for calculations
     */
    static int32_t rpm_to_counts_scaled(int rpm) {
        return static_cast<int32_t>((rpm * COUNTS_PER_REV * VELOCITY_FACTOR) / 60.0);
    }
    
    /**
     * @brief Convert encoder counts to RPM (corrected - no scaling factor)
     * @param counts Velocity in encoder counts per minute
     * @return Velocity in revolutions per minute
     * @note Uses raw encoder counts - no velocity factor applied
     */
    static int rpm_from_counts(int32_t counts) {
        return static_cast<int>(static_cast<double>(counts * 60) / COUNTS_PER_REV);
    }
    
    /**
     * @brief Convert encoder counts to RPM with precise calculation
     * @param counts Velocity in encoder counts per minute
     * @return Velocity in revolutions per minute (double precision)
     * @note Uses raw encoder counts - no velocity factor applied
     */
    static double rpm_from_counts_precise(int32_t counts) {
        return static_cast<double>(counts * 60) / COUNTS_PER_REV;
    }
    
    /**
     * @brief Convert encoder counts to RPM with velocity scaling (legacy)
     * @param counts Velocity in encoder counts per minute
     * @return Velocity in RPM using the velocity scaling factor
     * @note This version applies velocity scaling - kept for reference
     */
    static double rpm_from_counts_with_scaling(int32_t counts) {
        return static_cast<double>(counts * 60) / (COUNTS_PER_REV * VELOCITY_FACTOR);
    }
    
    /**
     * @brief Convert position change to output shaft RPM (accounts for gear reduction)
     * @param position_change Position change in encoder counts (motor shaft)
     * @param time_seconds Duration in seconds
     * @return Output shaft velocity in RPM
     * @note Position PDO reads motor shaft; this converts to output shaft RPM using 7.75:1 gear ratio
     */
    static double output_rpm_from_position_change(int32_t position_change, double time_seconds) {
        return (position_change * 60.0) / (COUNTS_PER_REV * GEAR_REDUCTION_RATIO * time_seconds);
    }
    
    /**
     * @brief Convert millinewton-meters to PDO torque units
     * @param millinm Torque in mNm
     * @return Torque in PDO units (per thousand of rated torque)
     */
    static uint16_t millinm_to_pdo(int16_t millinm) {
        return static_cast<uint16_t>((millinm * PDO_TORQUE_SCALE) / RATED_TORQUE_MNM);
    }
    
    /**
     * @brief Convert PDO torque units to millinewton-meters
     * @param pdo_value Torque in PDO units
     * @return Torque in mNm
     */
    static int16_t pdo_to_millinm(uint16_t pdo_value) {
        return static_cast<int16_t>((static_cast<int32_t>(pdo_value) * RATED_TORQUE_MNM) / PDO_TORQUE_SCALE);
    }
    
    /**
     * @brief Clamp torque value to safe operating limits
     * @param torque Input torque in mNm
     * @return Clamped torque within safe limits
     */
    static int16_t clamp_torque(int16_t torque) {
        if (torque > MAX_TORQUE_MILLINM) return MAX_TORQUE_MILLINM;
        if (torque < MIN_TORQUE_MILLINM) return MIN_TORQUE_MILLINM;
        return torque;
    }
};

} // namespace jd8
