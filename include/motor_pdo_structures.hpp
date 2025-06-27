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

namespace synapticon_motor {

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

// Forward declaration
class MotorConfigParser;

/**
 * @class MotorConstants
 * @brief Constants and utility functions for motor control
 * 
 * Contains motor-specific constants, conversion factors, and utility
 * functions. Motor specifications are now loaded from configuration files
 * to support different motor models (JD8, JD10, JD12, etc.).
 */
class MotorConstants {
public:
    // === Default Motor Specifications (fallbacks when no config available) ===
    static constexpr int DEFAULT_COUNTS_PER_REV = 524288;         ///< Default encoder counts per revolution
    static constexpr double DEFAULT_GEAR_REDUCTION_RATIO = 7.75;  ///< Default gear reduction ratio
    static constexpr int DEFAULT_MAX_VELOCITY_RPM = 315;         ///< Default maximum safe velocity in RPM
    static constexpr int16_t DEFAULT_RATED_TORQUE_MNM = 6000;    ///< Default rated torque: 6 Nm
    static constexpr int16_t DEFAULT_MAX_TORQUE_MILLINM = 3000;  ///< Default max safe torque: 3 Nm
    static constexpr int16_t DEFAULT_TORQUE_RAMP_RATE = 200;     ///< Default torque ramp rate
    
    // === Velocity Scaling ===
    static constexpr double RPM_SCALING_FACTOR = 0.001;   ///< Motor config 0x60A9: 0.001 RPM units
    
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
    
    // === Profile Position Mode Control Word Bits ===
    static constexpr uint16_t CONTROLWORD_NEW_SETPOINT = 0x0010;       ///< Bit 4: New set-point
    static constexpr uint16_t CONTROLWORD_CHANGE_SET_IMMEDIATELY = 0x0020;  ///< Bit 5: Change set immediately
    static constexpr uint16_t CONTROLWORD_POSITION_MODE = CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_NEW_SETPOINT | CONTROLWORD_CHANGE_SET_IMMEDIATELY;  ///< Complete position control word
    
    // === Profile Velocity Mode Control Word Bits ===
    static constexpr uint16_t CONTROLWORD_HALT = 0x0100;              ///< Bit 8: Halt (1=stop, 0=execute motion)
    static constexpr uint16_t CONTROLWORD_VELOCITY_MODE = CONTROLWORD_ENABLE_OPERATION;  ///< Velocity control word (halt bit clear)
    
    // === CIA402 Status Word Masks and Values ===
    static constexpr uint16_t STATUSWORD_STATE_MASK = 0x006F;         ///< State detection mask
    static constexpr uint16_t STATUSWORD_SWITCH_DISABLED_MASK = 0x004F; ///< Switch disabled mask
    static constexpr uint16_t STATUSWORD_OPERATION_ENABLED_VALUE = 0x0027; ///< Operation enabled state value
    static constexpr uint16_t STATUSWORD_SWITCH_ON_DISABLED_VALUE = 0x0040; ///< Switch on disabled state value
    static constexpr uint16_t STATUSWORD_READY_TO_SWITCH_ON = 0x0021;  ///< Ready to switch on
    static constexpr uint16_t STATUSWORD_SWITCHED_ON = 0x0023;         ///< Switched on
    static constexpr uint16_t STATUSWORD_OPERATION_ENABLED = 0x0237;   ///< Operation enabled
    static constexpr uint16_t STATUSWORD_FAULT = 0x0008;               ///< Fault detected
    
    // === Real-Time System Configuration ===
    static constexpr double RT_FREQUENCY_HZ = 250.0;         ///< RT kernel frequency: 250Hz
    static constexpr double CYCLE_TIME_MS = 4.0;             ///< RT cycle time: 4ms per cycle
    
    // === Object Dictionary Indices (CIA402 and Synapticon-specific) ===
    static constexpr uint16_t POSITION_GAINS_INDEX = 0x2010;      ///< Position PID gains
    static constexpr uint16_t VELOCITY_GAINS_INDEX = 0x2011;      ///< Velocity PID gains  
    static constexpr uint16_t CURRENT_GAINS_INDEX = 0x2012;       ///< Current PID gains
    static constexpr uint16_t MOTOR_CONFIG_INDEX = 0x2110;        ///< Motor configuration
    static constexpr uint16_t CIA402_MAX_SPEED_INDEX = 0x6080;    ///< CIA402 max motor speed
    static constexpr uint16_t CIA402_PROFILE_VELOCITY_INDEX = 0x6081; ///< CIA402 profile velocity (NOT resolution!)
    static constexpr uint16_t CIA402_POSITION_LIMITS_INDEX = 0x607D; ///< CIA402 position limits
    static constexpr uint16_t CIA402_TORQUE_LIMITS_INDEX = 0x6072;   ///< CIA402 torque limits
    
    
    // === Torque Control Constants ===
    static constexpr int16_t PDO_TORQUE_SCALE = 1000;        ///< PDO scale: 1000 = full torque
    
    // === Configuration-Driven Motor Parameters ===
    
    /**
     * @brief Get encoder counts per revolution from config
     * @param config Configuration parser (nullptr uses default)
     * @return Encoder counts per revolution
     */
    static uint32_t getCountsPerRev(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get gear reduction ratio from config
     * @param config Configuration parser (nullptr uses default)
     * @return Gear reduction ratio
     */
    static double getGearReductionRatio(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get rated torque from config
     * @param config Configuration parser (nullptr uses default)
     * @return Rated torque in mNm
     */
    static int16_t getRatedTorqueMNm(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get maximum torque from config
     * @param config Configuration parser (nullptr uses default)
     * @return Maximum torque in mNm
     */
    static int16_t getMaxTorqueMNm(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get minimum torque from config
     * @param config Configuration parser (nullptr uses default)
     * @return Minimum torque in mNm
     */
    static int16_t getMinTorqueMNm(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get torque ramp rate from config
     * @param config Configuration parser (nullptr uses default)
     * @return Torque ramp rate in mNm per cycle
     */
    static int16_t getTorqueRampRate(const MotorConfigParser* config = nullptr);
    
    /**
     * @brief Get maximum position change per cycle (config-driven)
     * @param config Configuration parser (nullptr uses default)
     * @return Maximum position change per 4ms cycle
     */
    static int32_t getMaxPositionChangePerCycle(const MotorConfigParser* config = nullptr);
    
    // === Velocity Conversions (0.001 RPM scaling per 0x60A9 config) ===
    
    /**
     * @brief Convert RPM to velocity PDO units 
     * @param rpm Velocity in revolutions per minute
     * @return Velocity PDO value (motor expects RPM_SCALING_FACTOR units)
     * @note Motor config 0x60A9 = 0.001 RPM, so 158 RPM → send 158000
     */
    static uint32_t rpm_to_velocity_pdo(int rpm) {
        return static_cast<uint32_t>(rpm / RPM_SCALING_FACTOR);
    }
    
    /**
     * @brief Convert velocity PDO to RPM
     * @param velocity_pdo_value Raw velocity PDO value from motor
     * @return Actual velocity in RPM
     * @note Motor config 0x60A9 = RPM_SCALING_FACTOR, so PDO * scaling = actual RPM
     */
    static double velocity_pdo_to_rpm(uint32_t velocity_pdo_value) {
        return static_cast<double>(static_cast<int32_t>(velocity_pdo_value)) * RPM_SCALING_FACTOR;
    }
    
    // === Position Conversions ===
    
    
    /**
     * @brief Convert position change to output shaft RPM (accounts for gear reduction)
     * @param position_change Position change in encoder counts (motor shaft)
     * @param time_seconds Duration in seconds
     * @param config Configuration parser (nullptr uses defaults)
     * @return Output shaft velocity in RPM
     * @note Position PDO reads motor shaft; this converts to output shaft RPM using gear ratio from config
     */
    static double output_rpm_from_position_change(int32_t position_change, double time_seconds, const MotorConfigParser* config = nullptr) {
        uint32_t counts_per_rev = getCountsPerRev(config);
        double gear_ratio = getGearReductionRatio(config);
        return (position_change * 60.0) / (counts_per_rev * gear_ratio * time_seconds);
    }
    
    /**
     * @brief Convert millinewton-meters to PDO torque units
     * @param millinm Torque in mNm
     * @param config Configuration parser (nullptr uses defaults)
     * @return Torque in PDO units (per thousand of rated torque)
     */
    static uint16_t millinm_to_pdo(int16_t millinm, const MotorConfigParser* config = nullptr) {
        int16_t rated_torque = getRatedTorqueMNm(config);
        return static_cast<uint16_t>((millinm * PDO_TORQUE_SCALE) / rated_torque);
    }
    
    /**
     * @brief Convert PDO torque units to millinewton-meters
     * @param pdo_value Torque in PDO units
     * @param config Configuration parser (nullptr uses defaults)
     * @return Torque in mNm
     */
    static int16_t pdo_to_millinm(uint16_t pdo_value, const MotorConfigParser* config = nullptr) {
        int16_t rated_torque = getRatedTorqueMNm(config);
        return static_cast<int16_t>((static_cast<int32_t>(pdo_value) * rated_torque) / PDO_TORQUE_SCALE);
    }
    
    /**
     * @brief Clamp torque value to safe operating limits
     * @param torque Input torque in mNm
     * @param config Configuration parser (nullptr uses defaults)
     * @return Clamped torque within safe limits
     */
    static int16_t clamp_torque(int16_t torque, const MotorConfigParser* config = nullptr) {
        int16_t max_torque = getMaxTorqueMNm(config);
        int16_t min_torque = getMinTorqueMNm(config);
        if (torque > max_torque) return max_torque;
        if (torque < min_torque) return min_torque;
        return torque;
    }
};

} // namespace synapticon_motor
