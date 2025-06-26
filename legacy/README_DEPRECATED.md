# DEPRECATED LEGACY CODE

⚠️ **WARNING: This directory contains DEPRECATED legacy code** ⚠️

## Status: OUTDATED - DO NOT USE

This legacy C implementation (`draco_test.c`) contains **incorrect scaling constants** and **deprecated motor control approaches** that are incompatible with the current JD8 controller implementation.

### Critical Issues:
- **Wrong encoder resolution**: Uses 4096 counts/rev instead of correct 524,288 counts/rev
- **Incorrect velocity scaling**: Uses deprecated factors incompatible with current 0.001 RPM scaling
- **Outdated control methods**: Does not implement gear ratio compensation or modern CIA402 state machine

### Use Instead:
- **Current C++ implementation**: `/src/jd8_controller.cpp`
- **Test programs**: `/tests/test_*.cpp`
- **API documentation**: `/include/jd8_controller.hpp`

### Historical Purpose:
This directory is preserved only for historical reference of the original implementation approach. All new development should use the modern C++ implementation in the main source tree.

**Last working version**: Unknown - predates current controller architecture
**Replacement**: Complete C++ implementation with proper scaling and safety features