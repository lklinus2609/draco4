/**
 * @file jd8_constants.cpp
 * @brief Implementation of JD8Constants config-driven functions
 * 
 * Provides motor-specific constants and conversion factors that are loaded
 * from configuration files to support different motor models.
 */

#include "motor_pdo_structures.hpp"
#include "motor_configuration.hpp"

namespace synapticon_motor {

uint32_t MotorConstants::getCountsPerRev(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return config->getEncoderResolution();
    }
    return DEFAULT_COUNTS_PER_REV;
}

double MotorConstants::getGearReductionRatio(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return config->getGearReductionRatio();
    }
    return DEFAULT_GEAR_REDUCTION_RATIO;
}

int16_t MotorConstants::getRatedTorqueMNm(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return config->getRatedTorqueMNm();
    }
    return DEFAULT_RATED_TORQUE_MNM;
}

int16_t MotorConstants::getMaxTorqueMNm(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return config->getMaxTorqueMNm();
    }
    return DEFAULT_MAX_TORQUE_MILLINM;
}

int16_t MotorConstants::getMinTorqueMNm(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return -config->getMaxTorqueMNm();  // Negative of max torque
    }
    return -DEFAULT_MAX_TORQUE_MILLINM;
}

int16_t MotorConstants::getTorqueRampRate(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        return config->getTorqueRampRate();
    }
    return DEFAULT_TORQUE_RAMP_RATE;
}

int32_t MotorConstants::getMaxPositionChangePerCycle(const MotorConfigParser* config) {
    if (config && config->isLoaded()) {
        uint32_t counts_per_rev = config->getEncoderResolution();
        // Allow 4 revolutions per cycle as maximum (conservative safety limit)
        return static_cast<int32_t>(4 * counts_per_rev);
    }
    // Default: 4 revolutions = 4 * 524288 = 2097152
    return 4 * DEFAULT_COUNTS_PER_REV;
}

} // namespace synapticon_motor