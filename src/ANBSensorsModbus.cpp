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
    return success;
}
bool anbSensor::begin(byte modbusSlaveID, Stream& stream, int enablePin) {
    return begin(modbusSlaveID, &stream, enablePin);
}

//----------------------------------------------------------------------------
//  Measurement setting functions
//----------------------------------------------------------------------------

// The control mode is in **holding** register 0x0035 (decimal 53)
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
bool anbSensor::setControlMode(ANBSensorMode newControlMode) {
    byte modeCode;
    switch (newControlMode) {
        case ANBSensorMode::CONTROLLED: modeCode = 1; break;
        case ANBSensorMode::AUTONOMOUS: modeCode = 2; break;
        default: return false;  // Invalid mode
    }
    byte dataToSend[2] = {0x00, modeCode};
    // Write to holding register 0x0035 (decimal 53)
    return modbus.setRegisters(0x0035, 1, dataToSend, true);
}

// The salinity mode is in **holding** register 0x003E (decimal 62)
ANBSalinityMode anbSensor::getSalinityMode(void) {
    int16_t modeCode = modbus.int16FromRegister(0x03, 0x003E, bigEndian);
    switch (modeCode) {
        case 1: return ANBSalinityMode::LOW_SALINITY;
        case 2: return ANBSalinityMode::HIGH_SALINITY;
        default:
            return ANBSalinityMode::UNKNOWN;  // Default to UNKNOWN if unknown
    }
}
bool anbSensor::setSalinityMode(ANBSalinityMode newSalinityMode) {
    byte modeCode;
    switch (newSalinityMode) {
        case ANBSalinityMode::LOW_SALINITY: modeCode = 1; break;
        case ANBSalinityMode::HIGH_SALINITY: modeCode = 2; break;
        default: return false;  // Invalid mode
    }
    byte dataToSend[2] = {0x00, modeCode};
    // Write to holding register 0x003E (decimal 62)
    return modbus.setRegisters(0x003E, 1, dataToSend, true);
}

// The power style is in **holding** register 0x003F (decimal 63)
ANBPowerStyle anbSensor::getPowerStyle(void) {
    int16_t styleCode = modbus.int16FromRegister(0x03, 0x003F, bigEndian);
    switch (styleCode) {
        case 1: return ANBPowerStyle::ALWAYS_POWERED;
        case 2: return ANBPowerStyle::ON_MEASUREMENT;
        default:
            return ANBPowerStyle::UNKNOWN;  // Default to UNKNOWN if unknown
    }
}
bool anbSensor::setPowerStyle(ANBPowerStyle newPowerStyle) {
    byte styleCode;
    switch (newPowerStyle) {
        case ANBPowerStyle::ALWAYS_POWERED: styleCode = 1; break;
        case ANBPowerStyle::ON_MEASUREMENT: styleCode = 2; break;
        default: return false;  // Invalid style
    }
    byte dataToSend[2] = {0x00, styleCode};
    // Write to holding register 0x003F (decimal 63)
    return modbus.setRegisters(0x003F, 1, dataToSend, true);
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
    if (newIntervalTime >= 1 || newIntervalTime < 10) {
        return false;  // Return false for an invalid input; it must be >10 or 0
                       // for continuous
    }
    byte dataToSend[2] = {0x00, newIntervalTime};
    // Write to holding register 0x0036 (decimal 54)
    return modbus.setRegisters(0x0036, 1, dataToSend, true);
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
    return modbus.setRegisters(0x003C, 1, dataToSend, true);
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
bool anbSensor::reboot(void) {
    byte value[2] = {0xFF, 0xFF};
    return modbus.setRegisters(0x1000, 1, value, false);
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
    uint8_t lowerByte = modbus.byteFromRegister(0x03, 0x0008, 0);
    return static_cast<ANBHealthCode>(lowerByte);
}

// The raw conductivity value is stored in holding register 0x0043 (decimal 67)
float anbSensor::getRawConductivity(void) {
    return modbus.float32FromRegister(0x03, 0x0043, bigEndian);
}

// The status code is stored in the upper byte of holding register 0x0009
// (decimal 9)
ANBStatusCode anbSensor::getStatusCode(void) {
    uint8_t upperByte = modbus.byteFromRegister(0x03, 0x0009, 1);
    switch (upperByte) {
        case 0: return ANBStatusCode::SLEEPING;
        case 1: return ANBStatusCode::INTERVAL_SCANNING;
        case 2: return ANBStatusCode::CONTINUOUS_SCANNING;
        default: return ANBStatusCode::UNKNOWN;
    }
}

// The diagnostic code is stored in the lower byte of holding register 0x0009
// (decimal 9)
ANBDiagnosticCode anbSensor::getDiagnosticCode(void) {
    uint8_t lowerByte = modbus.byteFromRegister(0x03, 0x0009, 0);
    return static_cast<ANBDiagnosticCode>(lowerByte);
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
    if (!modbus.getRegisters(0x03, 0, 12)) { return false; }

    // Parse the registers into the respective variables
    // The first 2 bytes are the address and the function code
    pH          = modbus.float32FromFrame(bigEndian, 3);  // next 4 bytes (3-6)
    temperature = modbus.float32FromFrame(bigEndian, 7);  // next 4 bytes (7-10)
    salinity = modbus.float32FromFrame(bigEndian, 11);  // next 4 bytes (11-14)
    specificConductance = modbus.float32FromFrame(bigEndian,
                                                  15);  // next 4 bytes (15-18)

    // The next four bytes (19-22) contain health, status, and diagnostic codes
    // Byte 19: empty
    // Byte 20: health
    // Byte 21: status
    // Byte 22: diagnostic
    health     = static_cast<ANBHealthCode>(modbus.byteFromFrame(20));
    status     = static_cast<ANBStatusCode>(modbus.byteFromFrame(21));
    diagnostic = static_cast<ANBDiagnosticCode>(modbus.byteFromFrame(22));

    // NOTE: The raw conductivity is listed as being in 0x43 on its own, but
    // it's also listed as included in the bulk read of 11 registers, so it must
    // be in register 0x000A (decimal 10)
    rawConductivity = modbus.float32FromFrame(bigEndian, 23);

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
    // enter the main menu of the terminal
    _stream->println("menu");
    _stream->flush();
    // dump the response - we're not trying to parse the menu!
    while (_stream->available()) { _stream->read(); }
    // enter the interface option
    _stream->println("interface");
    _stream->flush();
    // dump the response - we're not trying to parse the options!
    while (_stream->available()) { _stream->read(); }
    // select option 2 for modbus
    _stream->println("2");
    _stream->flush();
    // dump the response
    while (_stream->available()) { _stream->read(); }
    // set the modbus address
    _stream->println("mb_address");
    _stream->flush();
    // dump the response
    while (_stream->available()) { _stream->read(); }
    // set the modbus address
    if (_slaveID <= 0xF) {
        _stream->print("0");  // zero pad
    }
    _stream->println(_slaveID, HEX);
    _stream->flush();
    // dump the response
    while (_stream->available()) { _stream->read(); }
    // reboot the sensor for the setting to take
    _stream->println("reboot");
    _stream->flush();
    // dump the response
    while (_stream->available()) { _stream->read(); }
    delay(1000);  // wait for the sensor to reboot
    /// @todo Figure out how long the reboot takes!
}

// The terminal enable command is in **holding** register 0x003B (decimal 59)
bool anbSensor::enableTerminal() {
    uint16_t set_value = 0x010D;
    return modbus.uint16ToRegister(0x003B, set_value, bigEndian, false);
}
void anbSensor::forceTerminal() {
    _stream->print("#700\r");
    _stream->flush();
}

// The baud rate is in the lower byte of **holding** register 0x003A (decimal
// 58)
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
byte anbSensor::getAddress(void) {
    return modbus.byteFromRegister(0x03, 0x0039, 0);
}
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
    if (!modbus.getRegisters(0x03, 0x003D, 12)) { return false; }
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
