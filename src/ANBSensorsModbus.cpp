/**
 * @file ANBSensorsModbus.h
 * Part of the EnviroDIY ANBSensorsModbus library for Arduino.
 * @license This library is published under the BSD-3 license.
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the ANBSensorsModbus class definitions.
 *
 * Tested with AQ5
 */

#include "ANBSensorsModbus.h"

//----------------------------------------------------------------------------
//  Basic Communication Setup
//----------------------------------------------------------------------------

void anbSensor::setDefaultTimeouts() {
    modbus.setCommandTimeout(1000L);
    modbus.setFrameTimeout(10L);
    modbus.setCommandRetries(10);
}

anbSensor::anbSensor()
    : _slaveID(ANB_DEFAULT_MODBUS_ADDRESS),
      _stream(nullptr),
      modbus(static_cast<byte>(ANB_DEFAULT_MODBUS_ADDRESS), nullptr, -1) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(byte modbusSlaveID, Stream* stream)
    : _slaveID(modbusSlaveID),
      _stream(stream),
      modbus(modbusSlaveID, stream) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(byte modbusSlaveID, Stream& stream)
    : _slaveID(modbusSlaveID),
      _stream(&stream),
      modbus(modbusSlaveID, stream) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(byte modbusSlaveID, Stream* stream, int8_t enablePin)
    : _slaveID(modbusSlaveID),
      _stream(stream),
      modbus(modbusSlaveID, stream, enablePin) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(byte modbusSlaveID, Stream& stream, int8_t enablePin)
    : _slaveID(modbusSlaveID),
      _stream(&stream),
      modbus(modbusSlaveID, stream, enablePin) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(Stream* stream)
    : _slaveID(ANB_DEFAULT_MODBUS_ADDRESS),
      _stream(stream),
      modbus(static_cast<byte>(ANB_DEFAULT_MODBUS_ADDRESS), stream) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(Stream& stream)
    : _slaveID(ANB_DEFAULT_MODBUS_ADDRESS),
      _stream(&stream),
      modbus(static_cast<byte>(ANB_DEFAULT_MODBUS_ADDRESS), stream) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(Stream* stream, int8_t enablePin)
    : _slaveID(ANB_DEFAULT_MODBUS_ADDRESS),
      _stream(stream),
      modbus(static_cast<byte>(ANB_DEFAULT_MODBUS_ADDRESS), stream, enablePin) {
    setDefaultTimeouts();
}
anbSensor::anbSensor(Stream& stream, int8_t enablePin)
    : _slaveID(ANB_DEFAULT_MODBUS_ADDRESS),
      _stream(&stream),
      modbus(static_cast<byte>(ANB_DEFAULT_MODBUS_ADDRESS), stream, enablePin) {
    setDefaultTimeouts();
}

// This function sets up the communication
// It should be run during the arduino "setup" function.
// The "stream" device must be initialized and begun prior to running this.
bool anbSensor::begin(byte modbusSlaveID, Stream* stream, int enablePin) {
    // Give values to variables;
    _slaveID = modbusSlaveID;
    _stream  = stream;
    // Start up the modbus instance
    bool success = modbus.begin(modbusSlaveID, stream, enablePin);
    setDefaultTimeouts();
    return success;
}
bool anbSensor::begin(byte modbusSlaveID, Stream& stream, int enablePin) {
    return begin(modbusSlaveID, &stream, enablePin);
}
bool anbSensor::begin(byte modbusSlaveID, modbusMaster& modbus) {
    this->modbus  = modbus;
    this->_stream = this->modbus.getStream();
    this->modbus.setSlaveID(modbusSlaveID);
    _slaveID = modbusSlaveID;
    setDefaultTimeouts();
    return true;
}

// To check for a response, we will send the command to request status
// and look for any correctly formed modbus response.
bool anbSensor::gotModbusResponse(void) {
    modbus.setCommandRetries(1);
    getStatusCode();
    modbus.setCommandRetries(10);
    modbusErrorCode lastCode = modbus.getLastError();
    return static_cast<int8_t>(lastCode) < 0x0C;
}

// To check if the sensor is ready, we will send the command to request
// status and look for a modbus error code in response.
// NOTE: `getHealthCode()` and most of the data fetching commands will
// perpetually return an "ACKNOWLEDGE" error if the sensor is not measuring, so
// we cannot use those to check if the sensor is ready.
// `getStatusCode()` and metadata fetching commands (like `getSerialNumber()`)
// will return a valid response once the sensor is ready to communicate and
// begin measuring.
bool anbSensor::isSensorReady(void) {
    modbus.setCommandRetries(1);
    getStatusCode();
    modbus.setCommandRetries(10);
    return modbus.getLastError() == NO_ERROR;
}

// To check if a measurement is ready, we will send the command to request the
// health status and look for a modbus error code in response.
// NOTE: We don't use the returned health code to tell if the measurement is
// complete, just that the sensor responses and doesn't give any error code.
// If the measurement is not ready, the sensor will return `< ADDRESS >< 83 ><
// 06 >< CRC >` to indicate it's not ready or `< ADDRESS >< 83 >< 05 >< CRC >`
// to show it's not measuring at all.
bool anbSensor::isMeasurementComplete(void) {
    modbus.setCommandRetries(1);
    getHealthCode();
    modbus.setCommandRetries(10);
    return modbus.getLastError() == NO_ERROR;
}

//----------------------------------------------------------------------------
//  Measurement setting functions
//----------------------------------------------------------------------------

// The control mode is in **holding** register 0x0035 (decimal 53) and is write
// only
#if 0
// This function reads the control mode from the sensor and returns it as an
// ANBSensorMode enum.
ANBSensorMode anbSensor::getControlMode(void) {
    int16_t modeCode = modbus.int16FromRegister(0x03, 0x0035, bigEndian);
    switch (modeCode) {
        case 1: return ANBSensorMode::CONTROLLED;
        case 2: return ANBSensorMode::AUTONOMOUS;
        default:
            return ANBSensorMode::UNKNOWN;  // Default to UNKNOWN if
                                            // unknown
    }
}
#endif
bool anbSensor::setControlMode(ANBSensorMode newControlMode) {
    byte modeCode;
    switch (newControlMode) {
        case ANBSensorMode::CONTROLLED: modeCode = 1; break;
        case ANBSensorMode::AUTONOMOUS: modeCode = 2; break;
        default: return false;  // Invalid mode
    }
    byte dataToSend[2] = {0x00, modeCode};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x0035 (decimal 53)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0035, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}

// The salinity mode is in **holding** register 0x003E (decimal 62) and is write
// only.
#if 0
ANBSalinityMode anbSensor::getSalinityMode(void) {
    int16_t modeCode = modbus.int16FromRegister(0x03, 0x003E, bigEndian);
    switch (modeCode) {
        case 1: return ANBSalinityMode::LOW_SALINITY;
        case 2: return ANBSalinityMode::HIGH_SALINITY;
        default:
            return ANBSalinityMode::UNKNOWN;  // Default to UNKNOWN if unknown
    }
}
#endif
bool anbSensor::setSalinityMode(ANBSalinityMode newSalinityMode) {
    byte modeCode;
    switch (newSalinityMode) {
        case ANBSalinityMode::LOW_SALINITY: modeCode = 1; break;
        case ANBSalinityMode::HIGH_SALINITY: modeCode = 2; break;
        default: return false;  // Invalid mode
    }
    byte dataToSend[2] = {0x00, modeCode};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x003E (decimal 62)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x003E, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}

// The power style is in **holding** register 0x003F (decimal 63) and is  write
// only
#if 0
ANBPowerStyle anbSensor::getPowerStyle(void) {
    int16_t styleCode = modbus.int16FromRegister(0x03, 0x003F, bigEndian);
    switch (styleCode) {
        case 1: return ANBPowerStyle::ALWAYS_POWERED;
        case 2: return ANBPowerStyle::ON_MEASUREMENT;
        default:
            return ANBPowerStyle::UNKNOWN;  // Default to UNKNOWN if unknown
    }
}
#endif
bool anbSensor::setPowerStyle(ANBPowerStyle newPowerStyle) {
    byte styleCode;
    switch (newPowerStyle) {
        case ANBPowerStyle::ALWAYS_POWERED: styleCode = 1; break;
        case ANBPowerStyle::ON_MEASUREMENT: styleCode = 2; break;
        default: return false;  // Invalid style
    }
    byte dataToSend[2] = {0x00, styleCode};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x003F (decimal 63)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x003F, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}


// The interval time is stored in **holding** register 0x0036 (decimal 54)
uint8_t anbSensor::getIntervalTime(void) {
    int16_t interval = modbus.int16FromRegister(0x03, 0x0036, bigEndian);
    if (interval < 0 || interval > 255) {
        return 0;  // Return 0 if the value is out of expected range
    }
    return static_cast<uint8_t>(interval);
}
bool anbSensor::setIntervalTime(uint8_t newIntervalTime) {
    if (newIntervalTime >= 1 && newIntervalTime < 10) {
        return false;  // Return false for an invalid input; it must be >10 or 0
                       // for continuous
    }
    byte dataToSend[2] = {0x00, newIntervalTime};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x0036 (decimal 54)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0036, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}


#if 0
bool anbSensor::getStartDelay(uint16_t& hours, uint16_t& minutes) {
    if (!modbus.getRegisters(0x03, 0x0042, 2)) { return false; }

    // Parse the registers into the respective variables
    // The first 2 bytes are the address and the function code
    hours   = modbus.float32FromFrame(bigEndian, 3);  // next 4 bytes (3-6)
    minutes = modbus.float32FromFrame(bigEndian, 7);  // next 4 bytes (7-10)
    return true;
}
#endif
bool anbSensor::setStartDelay(uint16_t delayHours, uint16_t delayMinutes) {
    byte delayFrame[4];
    modbus.uint16ToFrame(delayHours, bigEndian, delayFrame, 0);
    modbus.uint16ToFrame(delayMinutes, bigEndian, delayFrame, 2);
    bool success = false;
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0042, 0x02, delayFrame, false);
        if (success) break;
    }
    return success;
}

// The immersion sensor status (immersion rule) is in **holding** register
// 0x003C (decimal 60)
bool anbSensor::isImmersionSensorEnabled(void) {
    int16_t status = modbus.int16FromRegister(0x03, 0x003C, bigEndian);
    return status == 1;
}
bool anbSensor::enableImmersionSensor(bool enable) {
    byte dataToSend[2] = {0x00, static_cast<byte>(enable ? 1 : 2)};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x003C (decimal 60)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x003C, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}

// The profiling mode is in **holding** register 0x0041 (decimal 65) and is
// write only
#if 0
bool anbSensor::isFastProfilingEnabled(void) {
    int16_t profileMode = modbus.int16FromRegister(0x03, 0x0041, bigEndian);
    switch (profileMode) {
        case 1: return true;
        case 2: return false;
        default:
            return false;  // Default to false if unknown
    }
}
#endif
bool anbSensor::enableFastProfiling(bool enable) {
    byte profileMode   = enable ? 1 : 2;
    byte dataToSend[2] = {0x00, profileMode};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x0041 (decimal 65)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0041, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}

// The SD card status is in **holding** register 0x0040 (decimal 64) and is
// write only
#if 0
bool anbSensor::isSDCardEnabled(void) {
    int16_t sdCardStatus = modbus.int16FromRegister(0x03, 0x0040, bigEndian);
    switch (sdCardStatus) {
        case 1: return true;
        case 2: return false;
        default:
            return false;  // Default to false if unknown
    }
}
#endif
bool anbSensor::enableSDCard(bool enable) {
    byte sdCardStatus  = enable ? 1 : 2;
    byte dataToSend[2] = {0x00, sdCardStatus};
    modbus.setCommandTimeout(5000L);
    bool success = false;
    // Write to holding register 0x0040 (decimal 64)
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0040, 1, dataToSend, false);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}


bool anbSensor::writeConfiguration(ANBSensorMode mode, ANBPowerStyle power,
                                   ANBSalinityMode salinity,
                                   uint16_t delayHours, uint16_t delayMinutes,
                                   uint16_t intervalHours,
                                   uint16_t intervalMinutes,
                                   bool profilingEnabled, bool modbusEnabled) {
    byte utilCmdFrame[16];
    byte controlCode;
    switch (mode) {
        case ANBSensorMode::CONTROLLED: controlCode = 1; break;
        case ANBSensorMode::AUTONOMOUS: controlCode = 2; break;
        default: return false;  // Invalid mode
    }
    byte styleCode;
    switch (power) {
        case ANBPowerStyle::ALWAYS_POWERED: styleCode = 1; break;
        case ANBPowerStyle::ON_MEASUREMENT: styleCode = 2; break;
        default: return false;  // Invalid style
    }
    byte salinityCode;
    switch (salinity) {
        case ANBSalinityMode::LOW_SALINITY: salinityCode = 1; break;
        case ANBSalinityMode::HIGH_SALINITY: salinityCode = 2; break;
        default: return false;  // Invalid mode
    }
    modbus.uint16ToFrame(controlCode, bigEndian, utilCmdFrame, 0);
    modbus.uint16ToFrame(styleCode, bigEndian, utilCmdFrame, 2);
    modbus.uint16ToFrame(salinityCode, bigEndian, utilCmdFrame, 4);
    modbus.uint16ToFrame(delayHours, bigEndian, utilCmdFrame, 6);
    modbus.uint16ToFrame(delayMinutes, bigEndian, utilCmdFrame, 8);
    modbus.uint16ToFrame(intervalHours, bigEndian, utilCmdFrame, 10);
    modbus.uint16ToFrame(intervalMinutes, bigEndian, utilCmdFrame, 12);
    modbus.uint16ToFrame(profilingEnabled ? 1 : 0, bigEndian, utilCmdFrame, 14);
    modbus.uint16ToFrame(modbusEnabled ? 1 : 0, bigEndian, utilCmdFrame, 16);
    bool success = false;
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0400, 0x08, utilCmdFrame, false);
        if (success) break;
    }
    return success;
}


//----------------------------------------------------------------------------
//  Command functions
//----------------------------------------------------------------------------

// For read-only Modbus data loggers, a scan is starting by sending the command
// {55 03 00 AA 00 0B[CRCL:CRCH]}
bool anbSensor::startReadOnly(void) {
    modbus.setCommandTimeout(5000L);
    int16_t bytesReturned = modbus.getModbusData(_slaveID, 0x03, 0x00AA, 0x000B,
                                                 4);
    modbus.setCommandTimeout(1000L);
    return bytesReturned == 4;
}

// The start scan command is set with **coil** 0x0100 (decimal 256)
bool anbSensor::start(void) {
    modbus.setCommandTimeout(5000L);
    bool success = modbus.setCoil(0x0100, true);
    modbus.setCommandTimeout(1000L);
    return success;
}

// The abrade sensor command is set with **coil** 0x0180 (decimal 384)
bool anbSensor::abradeSensor(void) {
    modbus.setCommandTimeout(5000L);
    bool success = modbus.setCoil(0x0180, true);
    modbus.setCommandTimeout(1000L);
    return success;
}

// The stop command is set with **coil** 0x0000
bool anbSensor::stop(void) {
    modbus.setCommandTimeout(5000L);
    bool success = modbus.setCoil(0x0000, true);
    modbus.setCommandTimeout(1000L);
    return success;
}

// The reboot command is set by writing 0xFFFF to **holding** register 0x1000
// (decimal 4096)
// NOTE: The response to the reboot command often has a junk 0x00 byte at the
// end, which will throw off the modbus response parsing.  Because of the junk
// byte, we turn off the retries for this command.
bool anbSensor::reboot(void) {
    if (_stream == nullptr) { return false; }
    // set the number of command retries to 1 (don't retry)
    modbus.setCommandRetries(1);
    modbus.setCommandTimeout(5000L);
    byte value[2] = {0xFF, 0xFF};
    modbus.setRegisters(0x1000, 1, value, false);
    // re-set the number of command retries to 10
    modbus.setCommandRetries(10);
    modbus.setCommandTimeout(1000L);
    /// @todo Figure out how long the reboot takes!
    delay(5000);
    // wait for the reboot menu print
    dumpToDebugStream();
    return gotModbusResponse();
}

bool anbSensor::forceReboot(bool alreadyInTerminal) {
    if (_stream == nullptr) { return false; }
    _stream->setTimeout(750);  // change timeout for the terminal commands
    // force enter terminal mode
    if (!alreadyInTerminal) { forceTerminal(); }
    // enter the main menu of the terminal
    delay(15);  // short delay before the command
    _stream->print("menu\r");
    _stream->flush();
    debugPrint("menu\n");
    // dump the response - we're not trying to parse the menu!
    dumpToDebugStream();
    // reboot the sensor for the setting to take
    delay(15);  // short delay before the command
    _stream->print("reboot\r");
    _stream->flush();
    debugPrint("reboot\n");
    /// @todo Figure out how long the reboot takes!
    dumpToDebugStream(10000L);
    _stream->setTimeout(1000L);  // restore the default timeout

    return gotModbusResponse();
}

// The shutdown command is set by writing 0x0000 to **holding** register 0x0200
// (decimal 512)
bool anbSensor::shutdown(void) {
    if (_stream == nullptr) { return false; }
    byte value[2] = {0x00, 0x00};
    bool success  = false;
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x0200, 1, value, false);
        if (success) break;
    }
    return gotModbusResponse();
}

//----------------------------------------------------------------------------
//  Measurement output functions
//----------------------------------------------------------------------------

// The pH value is stored in holding register 0x0000 (decimal 0)
float anbSensor::getpH(void) {
    return modbus.float32FromRegister(0x03, 0x0000, bigEndian);
}

// The temperature value is stored in holding register 0x0002 (decimal 2)
float anbSensor::getTemperature(void) {
    return modbus.float32FromRegister(0x03, 0x0002, bigEndian);
}

// The salinity value is stored in holding register 0x0004 (decimal 4)
float anbSensor::getSalinity(void) {
    return modbus.float32FromRegister(0x03, 0x0004, bigEndian);
}

// The specific conductance value is stored in holding register 0x0006 (decimal
// 6)
float anbSensor::getSpecificConductance(void) {
    return modbus.float32FromRegister(0x03, 0x0006, bigEndian);
}

// The health code is stored in the lower byte of holding register 0x0008
// (decimal 8)
ANBHealthCode anbSensor::getHealthCode(void) {
    uint8_t health = modbus.byteFromRegister(0x03, 0x0008, 0);
    switch (health) {
        case 0: return ANBHealthCode::OK;
        case 1: return ANBHealthCode::ABRADE_SOON;
        case 2: return ANBHealthCode::ABRADE_NOW;
        case 3: return ANBHealthCode::REPLACE;
        case 4: return ANBHealthCode::NOT_IMMERSED;
        case 5: return ANBHealthCode::NO_REFERENCE;
        case 6: return ANBHealthCode::NO_PH;
        default: return ANBHealthCode::UNKNOWN;
    }
}
String anbSensor::getHealthString(ANBHealthCode code) {
    switch (code) {
        case ANBHealthCode::OK: return "OK";
        case ANBHealthCode::ABRADE_SOON: return "Abrade Soon";
        case ANBHealthCode::ABRADE_NOW: return "Abrade Now";
        case ANBHealthCode::REPLACE: return "Replace";
        case ANBHealthCode::NOT_IMMERSED: return "Not Immersed";
        case ANBHealthCode::NO_REFERENCE: return "No Reference";
        case ANBHealthCode::NO_PH: return "No pH";
        default: return "Unknown";
    }
}
void anbSensor::printHealthCode(ANBHealthCode code, Stream& out) {
    out.print("Health Code: ");
    out.println(getHealthString(code));
}

// The raw conductivity value is stored in holding register 0x0043 (decimal 67)
float anbSensor::getRawConductivity(void) {
    return modbus.float32FromRegister(0x03, 0x0043, bigEndian);
}

// The status code is stored as the first digit of the two digit value stored in
// the lower byte of holding register 0x0009 (decimal 9).
ANBStatusCode anbSensor::getStatusCode(void) {
    uint8_t status_diag = modbus.byteFromRegister(0x03, 0x0009, 0);
    // bitwise and, then shift to get the first digit
    uint8_t status = (status_diag & 0xF0) >> 4;
    switch (status) {
        case 0: return ANBStatusCode::SLEEPING;
        case 1: return ANBStatusCode::INTERVAL_SCANNING;
        case 2: return ANBStatusCode::CONTINUOUS_SCANNING;
        default: return ANBStatusCode::UNKNOWN;
    }
}
String anbSensor::getStatusString(ANBStatusCode code) {
    switch (code) {
        case ANBStatusCode::SLEEPING: return "Sleeping";
        case ANBStatusCode::INTERVAL_SCANNING: return "Interval Scanning";
        case ANBStatusCode::CONTINUOUS_SCANNING: return "Continuous Scanning";
        default: return "Unknown";
    }
}
void anbSensor::printStatusCode(ANBStatusCode code, Stream& out) {
    out.print("Status Code: ");
    out.println(getStatusString(code));
}

// The diagnostic code is stored as the second digit of the two digit value
// stored in the lower byte of holding register 0x0009 (decimal 9).
ANBDiagnosticCode anbSensor::getDiagnosticCode(void) {
    uint8_t status_diag = modbus.byteFromRegister(0x03, 0x0009, 0);
    // bitwise and to get the second digit
    uint8_t diagnostic = status_diag & 0x0F;
    switch (diagnostic) {
        case 0: return ANBDiagnosticCode::OK;
        case 1: return ANBDiagnosticCode::BATTERY_ERROR;
        case 2: return ANBDiagnosticCode::SD_ERROR;
        case 3: return ANBDiagnosticCode::SYSTEM_ERROR;
        default: return ANBDiagnosticCode::UNKNOWN;
    }
}
String anbSensor::getDiagnosticString(ANBDiagnosticCode code) {
    switch (code) {
        case ANBDiagnosticCode::OK: return "OK";
        case ANBDiagnosticCode::BATTERY_ERROR: return "Battery Error";
        case ANBDiagnosticCode::SD_ERROR: return "SD Card Error";
        case ANBDiagnosticCode::SYSTEM_ERROR: return "System Error";
        default: return "Unknown";
    }
}
void anbSensor::printDiagnosticCode(ANBDiagnosticCode code, Stream& out) {
    out.print("Diagnostic Code: ");
    out.println(getDiagnosticString(code));
}

// All parameters can be read from 11 (0x0B) holding registers starting at
// 0x0000.
// BUT -- the documentation also says that pH, temp, salinity, conductivity, and
// actual conductivity can all be read from those registers.. which would
// require 12 registers.
// TODO: figure out what's up with the raw conductivity register location
bool anbSensor::getValues(float& pH, float& temperature, float& salinity,
                          float& specificConductance, float& rawConductivity,
                          ANBHealthCode&     health,
                          ANBDiagnosticCode& diagnostic) {
    if (!modbus.getRegisters(0x03, 0, 11)) { return false; }

    // NOTE: There are modbus map inconsistencies!
    // The sensor responds to the programmed modbus commands to give the values
    // as if they were in the expected registers, but it doesn't give the same
    // responses for each register when asking for registers in bulk.

    // In single commands the values are in these registers:
    // - 0x0000: pH (two registers)
    // - 0x0002: temperature (two registers)
    // - 0x0004: salinity (two registers)
    // - 0x0006: specific conductance (two registers)
    // - 0x0008: health (one register)
    // - 0x0009: status + diagnostic (one register)
    // - 0x000A: Serial number (3 registers)
    // - ... other things
    // - 0x0140: raw conductivity (two registers)

    // When pulling the registers starting from 0 in bulk, you get
    // - 0x0000: pH (two registers)
    // - 0x0002: temperature (two registers)
    // - 0x0004: salinity (two registers)
    // - 0x0006: specific conductance (two registers)
    // - 0x0008: health + diagnostic (both in one register - no status code!)
    // - 0x0009: raw conductivity (two registers)

    // Parse the registers into the respective variables
    // The first 2 bytes are the address and the function code
    pH          = modbus.float32FromFrame(bigEndian, 3);  // next 4 bytes (3-6)
    temperature = modbus.float32FromFrame(bigEndian, 7);  // next 4 bytes (7-10)
    salinity = modbus.float32FromFrame(bigEndian, 11);  // next 4 bytes (11-14)
    specificConductance = modbus.float32FromFrame(bigEndian,
                                                  15);  // next 4 bytes (15-18)

    // The next two bytes (19-20) contain health and diagnostic codes
    // NOTE: The status code is not included in this bulk read!
    // Byte 19: sensor diagnostics
    // Byte 20: transducer health
    diagnostic = static_cast<ANBDiagnosticCode>(modbus.byteFromFrame(19));
    health     = static_cast<ANBHealthCode>(modbus.byteFromFrame(20));

    rawConductivity = modbus.float32FromFrame(bigEndian, 21);

    return true;
}

//----------------------------------------------------------------------------
//  Admin functions
//----------------------------------------------------------------------------

// The modbus enable command is in **holding** register 0x0140 (decimal 320)
bool anbSensor::enableModbus() {
    modbus.setCommandTimeout(5000L);
    uint16_t set_value = 0x010D;
    bool success = modbus.uint16ToRegister(0x0140, set_value, bigEndian, false);
    modbus.setCommandTimeout(1000L);
    return success;
}
void anbSensor::forceModbus() {
    if (_stream == nullptr) { return; }
    _stream->setTimeout(750);  // change timeout for the terminal commands
    // clear anything hanging in the buffer
    dumpToDebugStream();
    forceTerminal();
    // enter the main menu of the terminal
    delay(15);  // short delay before the command
    _stream->print("menu\r");
    _stream->flush();
    debugPrint("menu\n");
    // dump the response - we're not trying to parse the menu!
    dumpToDebugStream();
    // enter the interface option
    delay(15);  // short delay before the command
    _stream->print("interface\r");
    _stream->flush();
    debugPrint("interface\n");
    // dump the response - we're not trying to parse the options!
    dumpToDebugStream();
    // select option 2 for modbus
    delay(15);  // short delay before the command
    _stream->print("2\r");
    _stream->flush();
    debugPrint("2\n");
    // dump the response
    dumpToDebugStream();
    // set the modbus address
    delay(15);  // short delay before the command
    _stream->print("mb_address\r");
    _stream->flush();
    debugPrint("mb_address\n");
    // dump the response
    dumpToDebugStream();
    // set the modbus address
    delay(15);  // short delay before the command
    if (_slaveID <= 0xF) {
        _stream->print("0");  // zero pad
        debugPrint("0");
    }
    _stream->print(_slaveID, HEX);
    _stream->print('\r');
    _stream->flush();
    debugPrint(String(_slaveID, HEX), '\n');
    // dump the response
    dumpToDebugStream();
    // reboot the sensor for the setting to take
    forceReboot(true);
    _stream->setTimeout(1000L);  // restore the default timeout
}

// The terminal enable command is in **holding** register 0x003B (decimal 59)
bool anbSensor::enableTerminal() {
    modbus.setCommandTimeout(5000L);
    uint16_t set_value = 0x0001;
    bool success = modbus.uint16ToRegister(0x003B, set_value, bigEndian, false);
    modbus.setCommandTimeout(1000L);
    return success;
}
void anbSensor::forceTerminal() {
    if (_stream == nullptr) { return; }
    // clear anything hanging in the buffer
    dumpToDebugStream();
    delay(15);             // short delay before the command
    _stream->print("\r");  // send a naked carriage return to clear the input
                           // command buffer
    _stream->flush();
    debugPrint("\n");
    // clear any error response, if it's there
    dumpToDebugStream();
    delay(15);  // short delay before the command
    _stream->print("#700\r");
    _stream->flush();
    debugPrint("#700\n");
    // dump the response: Sensor Setup functions are now available
    dumpToDebugStream();
}

// The baud rate is in the lower byte of **holding** register 0x003A (decimal
// 58) and is write only
#if 0
ANBSensorBaud anbSensor::getBaud(void) {
    uint8_t baud_code = modbus.byteFromRegister(0x03, 0x003A, 0);
    switch (baud_code) {
        case 1: return ANBSensorBaud::BAUD9600;
        case 2: return ANBSensorBaud::BAUD14400;
        case 3: return ANBSensorBaud::BAUD19200;
        case 4: return ANBSensorBaud::BAUD28800;
        case 5: return ANBSensorBaud::BAUD38400;
        case 6: return ANBSensorBaud::BAUD56000;
        case 7: return ANBSensorBaud::BAUD57600;
        case 8: return ANBSensorBaud::BAUD115200;
        default: return ANBSensorBaud::UNKNOWN;
    }
}
#endif
bool anbSensor::setBaud(ANBSensorBaud newSensorBaud) {
    uint8_t baud_code;
    switch (newSensorBaud) {
        case ANBSensorBaud::BAUD9600: baud_code = 1; break;
        case ANBSensorBaud::BAUD14400: baud_code = 2; break;
        case ANBSensorBaud::BAUD19200: baud_code = 3; break;
        case ANBSensorBaud::BAUD28800: baud_code = 4; break;
        case ANBSensorBaud::BAUD38400: baud_code = 5; break;
        case ANBSensorBaud::BAUD56000: baud_code = 6; break;
        case ANBSensorBaud::BAUD57600: baud_code = 7; break;
        case ANBSensorBaud::BAUD115200: baud_code = 8; break;
        default: return false;  // Unknown baud rate
    }
    modbus.setCommandTimeout(5000L);
    // Write the baud rate code to the lower byte of **holding** register 0x003A
    // (decimal 58)
    bool success = modbus.byteToRegister(0x3A, 0, baud_code);
    modbus.setCommandTimeout(1000L);
    return success;
}

// The address is in the lower byte of **holding** register 0x0039 (decimal 57)
// and is write only
#if 0
byte anbSensor::getAddress(void) {
    return modbus.byteFromRegister(0x03, 0x0039, 0);
}
#endif
bool anbSensor::setAddress(byte newSensorAddress) {
    if (newSensorAddress == 0 || newSensorAddress == 35 ||
        newSensorAddress > 247) {
        // Modbus addresses must be between 1 and 247, but 0x23 (35) is reserved
        // on ANB sensors
        return false;
    }
    modbus.setCommandTimeout(5000L);
    // Write the new address to the lower byte of **holding** register 0x0039
    bool success = modbus.byteToRegister(0x39, 0, newSensorAddress);
    modbus.setCommandTimeout(1000L);
    return success;
}

// The serial number takes up 3 holding registers starting at 0x000A (decimal
// 10)
String anbSensor::getSerialNumber(void) {
    return modbus.StringFromRegister(3, 0x000A, 6);
}

// The manufacturer information takes up 8 holding registers starting at 0x000D
// (decimal 13)
String anbSensor::getManufacturer(void) {
    return modbus.StringFromRegister(3, 0x000D, 16);
}

// The sensor name takes up 8 holding registers starting at 0x0015 (decimal 21)
String anbSensor::getName(void) {
    return modbus.StringFromRegister(3, 0x0015, 16);
}

// The sensor sub-name takes up 8 holding registers starting at 0x001D (decimal
// 29)
String anbSensor::getSubName(void) {
    return modbus.StringFromRegister(3, 0x001D, 16);
}

// The interface firmware (IF) version takes up 8 holding registers starting at
// 0x0025 (decimal 37)
String anbSensor::getInterfaceVersion(void) {
    return modbus.StringFromRegister(3, 0x0025, 16);
}

// The driver (DV) firmware version takes up 8 holding registers starting at
// 0x002D (decimal 45)
String anbSensor::getDriverVersion(void) {
    return modbus.StringFromRegister(3, 0x002D, 16);
}

// The RTC value is stored in BCD (Binary Coded Decimal) in 6 holding registers
// starting at 0x003D (decimal 61)
// NOTE: The values are stored as packed BCD, so each byte contains two digits
// and only the lower byte of each register is used.
// NOTE: The year is stored as an offset from 2000
bool anbSensor::getRTC(int8_t& seconds, int8_t& minutes, int8_t& hours,
                       int8_t& day, int8_t& month, int16_t& year) {
    if (!modbus.getRegisters(0x03, 0x003D, 6)) { return false; }
    seconds = bcd2dec(modbus.byteFromFrame(4));
    minutes = bcd2dec(modbus.byteFromFrame(6));
    hours   = bcd2dec(modbus.byteFromFrame(8));
    day     = bcd2dec(modbus.byteFromFrame(10));
    month   = bcd2dec(modbus.byteFromFrame(12));
    year    = bcd2dec(modbus.byteFromFrame(14)) + 2000;
    return true;
}
bool anbSensor::setRTC(int8_t seconds, int8_t minutes, int8_t hours, int8_t day,
                       int8_t month, int16_t year) {
    // Write the RTC values to the holding registers starting at 0x003D
    byte time_bytes[12];
    memset(time_bytes, 0, 12);
    time_bytes[1]  = dec2bcdRTC(seconds);
    time_bytes[3]  = dec2bcdRTC(minutes);
    time_bytes[5]  = dec2bcdRTC(hours);
    time_bytes[7]  = dec2bcdRTC(day);
    time_bytes[9]  = dec2bcdRTC(month);
    time_bytes[11] = dec2bcd(year - 2000);
    modbus.setCommandTimeout(5000L);
    bool success = false;
    for (size_t i = 0; i < 5; i++) {
        success = modbus.setRegisters(0x003D, 6, time_bytes);
        if (success) break;
    }
    modbus.setCommandTimeout(1000L);
    return success;
}
