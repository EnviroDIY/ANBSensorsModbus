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

// This function sets up the communication
// It should be run during the arduino "setup" function.
// The "stream" device must be initialized and begun prior to running this.
bool anbSensor::begin(byte modbusSlaveID, Stream* stream, int enablePin) {
    // Give values to variables;
    _slaveID = modbusSlaveID;
    _stream  = stream;
    // Start up the modbus instance
    bool success = modbus.begin(modbusSlaveID, stream, enablePin);
    // increase the wait time for a response to a modbus command from the
    // default of 500ms to 5s
    modbus.setCommandTimeout(5000L);
    // increase the wait for the next byte mid-frame from 4ms to 10ms
    modbus.setFrameTimeout(10L);
    // set the number of command retries to 10 (the default)
    modbus.setCommandRetries(10);
    return success;
}
bool anbSensor::begin(byte modbusSlaveID, Stream& stream, int enablePin) {
    return begin(modbusSlaveID, &stream, enablePin);
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
    // Write to holding register 0x0035 (decimal 53)
    return modbus.setRegisters(0x0035, 1, dataToSend, false);
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
    // Write to holding register 0x003E (decimal 62)
    return modbus.setRegisters(0x003E, 1, dataToSend, false);
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
    // Write to holding register 0x003F (decimal 63)
    return modbus.setRegisters(0x003F, 1, dataToSend, false);
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
    // Write to holding register 0x0036 (decimal 54)
    return modbus.setRegisters(0x0036, 1, dataToSend, false);
}

// The immersion sensor status (immersion rule) is in **holding** register
// 0x003C (decimal 60)
bool anbSensor::isImmersionSensorEnabled(void) {
    int16_t status = modbus.int16FromRegister(0x03, 0x003C, bigEndian);
    return status == 1;
}
bool anbSensor::enableImmersionSensor(bool enable) {
    byte dataToSend[2] = {0x00, static_cast<byte>(enable ? 1 : 2)};
    // Write to holding register 0x003C (decimal 60)
    return modbus.setRegisters(0x003C, 1, dataToSend, false);
}


//----------------------------------------------------------------------------
//  Command functions
//----------------------------------------------------------------------------

// The start scan command is set with **coil** 0x0100 (decimal 256)
bool anbSensor::start(void) {
    return modbus.setCoil(0x0100, true);
}

// The abrade sensor command is set with **coil** 0x0180 (decimal 384)
bool anbSensor::abradeSensor(void) {
    return modbus.setCoil(0x0180, true);
}

// The stop command is set with **coil** 0x0000
bool anbSensor::stop(void) {
    return modbus.setCoil(0x0000, true);
}

// The reboot command is set by writing 0xFFFF to **holding** register 0x1000
// (decimal 4096)
// NOTE: The response to the reboot command often has a junk 0x00 byte at the
// end, which will throw off the modbus response parsing.  Because of the junk
// byte, we turn off the retries for this command.
bool anbSensor::reboot(void) {
    // set the number of command retries to 1 (don't retry)
    modbus.setCommandRetries(1);
    byte value[2] = {0xFF, 0xFF};
    modbus.setRegisters(0x1000, 1, value, false);
    // re-set the number of command retries to 10
    modbus.setCommandRetries(1);
    // wait for the reboot menu print
    uint32_t startTime = millis();
    while (millis() - startTime < 10000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
}

// To check if a measurement is complete, we will send the command to request
// status and look for a modbus error code in response.
// NOTE: We don't use the returned health code to tell if the measurement is
// complete, just that the sensor responses and doesn't give an error code.
bool anbSensor::isMeasurementComplete(void) {
    getHealthCode();
    return modbus.getLastError() == NO_ERROR;
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

// All parameters can be read from 11 (0x0B) holding registers starting at
// 0x0000.
// BUT -- the documentation also says that pH, temp, salinity, conductivity, and
// actual conductivity can all be read from those registers.. which would
// require 12 registers.
// TODO: figure out what's up with the raw conductivity register location
bool anbSensor::getValues(float& pH, float& temperature, float& salinity,
                          float& specificConductance, float& rawConductivity,
                          ANBHealthCode& health, ANBStatusCode& status,
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
    // - 0x0008: health + status + diagnostic (all in one register)
    // - 0x0009: raw conductivity (two registers)

    // Parse the registers into the respective variables
    // The first 2 bytes are the address and the function code
    pH          = modbus.float32FromFrame(bigEndian, 3);  // next 4 bytes (3-6)
    temperature = modbus.float32FromFrame(bigEndian, 7);  // next 4 bytes (7-10)
    salinity = modbus.float32FromFrame(bigEndian, 11);  // next 4 bytes (11-14)
    specificConductance = modbus.float32FromFrame(bigEndian,
                                                  15);  // next 4 bytes (15-18)

    // The next two bytes (19-20) contain health, status, and diagnostic codes
    // Byte 19: health
    // Byte 20: status + diagnostic
    health              = static_cast<ANBHealthCode>(modbus.byteFromFrame(19));
    uint8_t status_diag = modbus.byteFromFrame(20);
    // bitwise and, then shift to get the first digit
    status     = static_cast<ANBStatusCode>((status_diag & 0xF0) >> 4);
    diagnostic = static_cast<ANBDiagnosticCode>(status_diag & 0x0F);

    rawConductivity = modbus.float32FromFrame(bigEndian, 21);

    return true;
}

//----------------------------------------------------------------------------
//  Admin functions
//----------------------------------------------------------------------------

// The modbus enable command is in **holding** register 0x0140 (decimal 320)
bool anbSensor::enableModbus() {
    uint16_t set_value = 0x010D;
    return modbus.uint16ToRegister(0x0140, set_value, bigEndian, false);
}
void anbSensor::forceModbus() {
    uint32_t currentTimeout =
        _stream->getTimeout();  // save the current timeout
    _stream->setTimeout(750);  // set a longer timeout for the terminal commands
    // clear anything hanging in the buffer
    do { _stream->readString(); } while (_stream->available());
    forceTerminal();
    // enter the main menu of the terminal
    delay(15);  // short delay before the command
    _stream->print("menu\r");
    _stream->flush();
    // dump the response - we're not trying to parse the menu!
    uint32_t startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    // enter the interface option
    delay(15);  // short delay before the command
    _stream->print("interface\r");
    _stream->flush();
    // dump the response - we're not trying to parse the options!
    startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    // select option 2 for modbus
    delay(15);  // short delay before the command
    _stream->print("2\r");
    _stream->flush();
    // dump the response
    startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    // set the modbus address
    delay(15);  // short delay before the command
    _stream->print("mb_address\r");
    _stream->flush();
    // dump the response
    startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    // set the modbus address
    delay(15);  // short delay before the command
    if (_slaveID <= 0xF) {
        _stream->print("0");  // zero pad
    }
    _stream->print(_slaveID, HEX);
    _stream->print('\r');
    _stream->flush();
    // dump the response
    startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    // reboot the sensor for the setting to take
    delay(15);  // short delay before the command
    _stream->print("reboot\r");
    _stream->flush();
    // dump the response
    startTime = millis();
    while (millis() - startTime < 10000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
    /// @todo Figure out how long the reboot takes!
    _stream->setTimeout(currentTimeout);  // restore the original timeout
}

// The terminal enable command is in **holding** register 0x003B (decimal 59)
bool anbSensor::enableTerminal() {
    uint16_t set_value = 0x010D;
    return modbus.uint16ToRegister(0x003B, set_value, bigEndian, false);
}
void anbSensor::forceTerminal() {
    // clear anything hanging in the buffer
    while (_stream->available()) { _stream->readString(); }
    delay(15);             // short delay before the command
    _stream->print("\r");  // send a naked carriage return to clear the input
                           // command buffer
    _stream->flush();
    // clear any error response, if it's there
    do { _stream->readString(); } while (_stream->available());
    delay(15);  // short delay before the command
    _stream->print("#700\r");
    _stream->flush();
    // dump the response: Sensor Setup functions are now available
    uint32_t startTime = millis();
    while (millis() - startTime < 5000L && _stream->available() < 10);
    do { _stream->readString(); } while (_stream->available());
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
    // Write the baud rate code to the lower byte of **holding** register 0x003A
    // (decimal 58)
    return modbus.byteToRegister(0x3A, 0, baud_code);
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
    // Write the new address to the lower byte of **holding** register 0x0039
    return modbus.byteToRegister(0x39, 0, newSensorAddress);
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

// The RTC value is stored in 6 holding registers starting at 0x003D (decimal
// 61)
bool anbSensor::getRTC(uint8_t& ss, uint8_t& mm, uint8_t& hh, uint8_t& dd,
                       uint8_t& MM, uint8_t& yy) {
    if (!modbus.getRegisters(0x03, 0x003D, 6)) { return false; }
    ss = modbus.byteFromFrame(3);
    mm = modbus.byteFromFrame(4);
    hh = modbus.byteFromFrame(5);
    dd = modbus.byteFromFrame(6);
    MM = modbus.byteFromFrame(7);
    yy = modbus.byteFromFrame(8);
    return true;
}
bool anbSensor::setRTC(uint8_t ss, uint8_t mm, uint8_t hh, uint8_t dd,
                       uint8_t MM, uint8_t yy) {
    // Write the RTC values to the holding registers starting at 0x003D
    byte rtc_values[6] = {ss, mm, hh, dd, MM, yy};
    return modbus.setRegisters(0x003D, 6, rtc_values);
}
