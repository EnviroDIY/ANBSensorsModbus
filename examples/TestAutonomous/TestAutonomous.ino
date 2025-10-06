/** =========================================================================
 * @example{lineno} TestAutonomous.ino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @license This example is published under the BSD-3 license.
 *
 * @brief This prints basic meta-data about a sensor to the first serial port
 * and then begins taking measurements from the sensor.
 *
 * @m_examplenavigation{example_get_values,}
 * @m_footernavigation
 * ======================================================================= */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
#include <Arduino.h>
#include <ANBSensorsModbus.h>

#include <SparkFun_RV8803.h>  //Get the library here:http://librarymanager/All#SparkFun_RV-8803
// To run in autonomous mode, the sensor's internal RTC should be set. This
// example uses a RV-8803 RTC to set the sensor clock. The time on the RV-8803
// should be set independently.
RV8803 rtc;

// ==========================================================================
//  Sensor Settings
// ==========================================================================

// Define the sensor's modbus address, or SlaveID
byte modbusAddress = 0x55;  // HEX 0x55 is the ANB default modbus address.

// The Modbus baud rate the sensor uses
int32_t modbusBaud = 115200;  // 57600 is ANB default baud rate.

// Sensor Timing
// Edit these to explore
#define WARM_UP_TIME 30000L      // milliseconds to wait for sensor to respond
#define STABILIZATION_TIME 100   // milliseconds for readings to stabilize.
#define MEASUREMENT_TIME 300000  // milliseconds to complete a measurement.


// ==========================================================================
//  Data Logger Options
// ==========================================================================
const int32_t serialBaud = 115200;  // Baud rate for serial monitor

// Define pin number variables
const int sensorPwrPin  = 56;  // The pin sending power to the sensor
const int adapterPwrPin = -1;  // The pin sending power to the RS485 adapter
const int DEREPin =
    -1;  // The pin controlling Receive Enable and Driver Enable
         // on the RS485 adapter, if applicable (else, -1)
         // Setting HIGH enables the driver (arduino) to send text
         // Setting LOW enables the receiver (sensor) to send text

ANBSalinityMode salinity   = ANBSalinityMode::LOW_SALINITY;
uint16_t        delayHours = 0;  // hours to wait before starting measurements
uint16_t delayMinutes      = 0;  // minutes to wait before starting measurements
uint16_t intervalHours     = 0;  // hours between measurements
uint16_t intervalMinutes   = 0;  // minutes between measurements
// set both hours and minute to 0 for continuous measurements
bool profilingEnabled = false;  // true to enable fast profiling
bool modbusEnabled    = true;   // true to enable modbus

// Turn on debugging outputs (i.e. raw Modbus requests & responses)
// by uncommenting next line (i.e. `#define DEBUG`)
#define DEBUG

// ==========================================================================
// Create and Assign a Serial Port for Modbus
// ==========================================================================
// Hardware serial ports are preferred when available.
// AltSoftSerial is the most stable alternative for modbus.
// Select over alternatives with the define below.
#define BUILD_ALTSOFTSERIAL
#if defined(BUILD_ALTSOFTSERIAL) && defined(__AVR__)
#include <AltSoftSerial.h>
AltSoftSerial modbusSerial;

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P)
// The Uno only has 1 hardware serial port, which is dedicated to communication
// with the computer. If using an Uno, you will be restricted to using
// AltSofSerial or SoftwareSerial
#include <SoftwareSerial.h>
const int SSRxPin =
    10;  // Receive pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 11;  // Send pin for software serial (Tx on RS485 adapter)
#pragma message("Using Software Serial for the Uno on pins 10 and 11")
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);

#elif defined(ESP8266)
#include <SoftwareSerial.h>
#pragma message("Using Software Serial for the ESP8266")
SoftwareSerial modbusSerial;

#elif defined(NRF52832_FEATHER) || defined(ARDUINO_NRF52840_FEATHER)
#pragma message("Using TinyUSB for the NRF52")
#include <Adafruit_TinyUSB.h>
HardwareSerial& modbusSerial = Serial1;

#elif !defined(NO_GLOBAL_SERIAL1) && !defined(STM32_CORE_VERSION)
// This is just a assigning another name to the same port, for convenience
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HardwareSerial / Serial1")
HardwareSerial& modbusSerial = Serial1;

#else
// This is just a assigning another name to the same port, for convenience
// Unless it is unavailable, always prefer hardware serial.
#pragma message("Using HardwareSerial / Serial")
HardwareSerial& modbusSerial = Serial;
#endif

// Construct the anbSensor instance
anbSensor sensor;
bool      success;
uint32_t  startTime;
uint32_t  readingNum = 0;

// ==========================================================================
// Working Functions
// ==========================================================================
// A function for pretty-printing the Modbuss Address in Hexadecimal notation,
// from ANBSensorsModbus `sensorLocation()`
String prettyprintAddressHex(byte _modbusAddress) {
    String addressHex = F("0x");
    if (_modbusAddress < 0x10) { addressHex += "0"; }
    addressHex += String(_modbusAddress, HEX);
    return addressHex;
}

void setSensorPower(bool power) {
    if (sensorPwrPin >= 0) { digitalWrite(sensorPwrPin, power ? HIGH : LOW); }
    if (adapterPwrPin >= 0 && adapterPwrPin != sensorPwrPin) {
        digitalWrite(adapterPwrPin, power ? HIGH : LOW);
    }
}

bool powerCycleSensor() {
    Serial.print(F("Holding with power off for 5s"));
    setSensorPower(false);
    for (size_t i = 0; i < 5; i++) {
        delay(1000L);
        Serial.print('.');
    }
    Serial.println();

    // Turn on power pins
    Serial.println(F("Powering on"));
    setSensorPower(true);

    // Allow the sensor and converter to warm up
    Serial.print(F("Waiting up to "));
    Serial.print(WARM_UP_TIME);
    Serial.println(F(" ms for the sensor to be ready... "));
    bool     isReady   = false;
    uint32_t startTime = millis();
    do {
        delay(250);
        isReady = sensor.isSensorReady();
    } while (!isReady && millis() - startTime <= WARM_UP_TIME);
    if (isReady) {
        Serial.print(F(" ...Sensor ready after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" ms"));
    } else {
        Serial.print(F("Timed out waiting for ready after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" ms"));
    }
    return isReady;
}

// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
// Wait for USB connection to be established by PC
// NOTE:  Only use this when debugging - if not connected to a PC, this adds an
// unnecessary startup delay
#if defined(SERIAL_PORT_USBVIRTUAL)
    do { delay(100); } while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000L));
#endif

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(serialBaud);

    // Set pin modes
    if (sensorPwrPin >= 0) { pinMode(sensorPwrPin, OUTPUT); }
    if (adapterPwrPin >= 0 && adapterPwrPin != sensorPwrPin) {
        pinMode(adapterPwrPin, OUTPUT);
    }
    if (DEREPin >= 0) { pinMode(DEREPin, OUTPUT); }
#if defined(LED_BUILTIN)
    if (LED_BUILTIN >= 0) { pinMode(LED_BUILTIN, OUTPUT); }
#endif

    // Turn on your modbus serial port
#if defined(ESP8266)
    const int SSRxPin =
        13;  // Receive pin for software serial (Rx on RS485 adapter)
    const int SSTxPin =
        14;  // Send pin for software serial (Tx on RS485 adapter)
    modbusSerial.begin(modbusBaud, SWSERIAL_8N1, SSRxPin, SSTxPin, false);
#else  // For Hardware Serial
    modbusSerial.begin(modbusBaud);
#endif

    // Start up the connection to the ANB pH sensor
    sensor.begin(modbusAddress, &modbusSerial, DEREPin);

// Turn on debugging
#ifdef DEBUG
    sensor.setDebugStream(&Serial);
#endif

    // Start up note
    Serial.print(F("\nTesting ANB pH Sensor"));
    Serial.println();

    // Confirm Modbus Address
    Serial.println(F("Selected modbus address:"));
    Serial.print(F("  Decimal: "));
    Serial.print(modbusAddress, DEC);
    Serial.print(F(", Hexidecimal: "));
    Serial.println(prettyprintAddressHex(modbusAddress));
    Serial.println();

    bool isReady = powerCycleSensor();

    if (!isReady) {
        Serial.println(F("Did not find the sensor."));
        Serial.println(F("Use ANB Sensors Utility to confirm the sensor is in "
                         "Modbus mode."));
        Serial.println(F("Check wiring and modbus address and try again."));
        while (1);  // Stay here forevermore
    }

    // // Enable modbus
    // Serial.println(F("Enabling Modbus... "));
    // bool isEnabled = sensor.enableModbus();
    // Serial.print(F(" ..."));
    // Serial.println(isEnabled ? F("success") : F("failed"));

    // Try to force modbus if the enable failed
    if (!isReady) {
        Serial.println(F("Trying to get any modbus response... "));
        bool gotModbusResponse = sensor.gotModbusResponse();
        if (!gotModbusResponse) {
            Serial.println(F("Did not get a modbus response, trying to force "
                             "Modbus enable... "));
            sensor.forceModbus();
            Serial.println(F("Trying again get a modbus response... "));
            gotModbusResponse = sensor.gotModbusResponse();
            Serial.print(F(" ..."));
            Serial.println(gotModbusResponse ? F("success") : F("failed"));
        }
    }

    // Get Sensor Information
    Serial.print(F("\n\n\nGet sensor information...\n"));

    // Get the sensor serial number
    Serial.println(F("Getting sensor serial number."));
    String SN = sensor.getSerialNumber();
    Serial.print(F("    Serial Number: "));
    Serial.println(SN);

    // Get the sensor manufacturer
    Serial.println(F("Getting sensor manufacturer."));
    String manufacturer = sensor.getManufacturer();
    Serial.print(F("    Manufacturer: "));
    Serial.println(manufacturer);

    // Get the sensor name
    Serial.println(F("Getting sensor name."));
    String name = sensor.getName();
    Serial.print(F("    Name: "));
    Serial.println(name);

    // Get the sensor sub-name
    Serial.println(F("Getting sensor sub-name."));
    String subname = sensor.getSubName();
    Serial.print(F("    Subname: "));
    Serial.println(subname);

    // Get the sensor interface version
    Serial.println(F("Getting sensor interface version."));
    String interfaceVersion = sensor.getInterfaceVersion();
    Serial.print(F("    Interface Version: "));
    Serial.println(interfaceVersion);

    // Get the sensor driver version
    Serial.println(F("Getting sensor driver version."));
    String driverVersion = sensor.getDriverVersion();
    Serial.print(F("    Driver Version: "));
    Serial.println(driverVersion);

    // Set the sensor RTC
    Wire.begin();
    rtc.begin();
    rtc.updateTime();
    Serial.println(F("Setting sensor RTC."));
    sensor.setRTC(rtc.getSeconds(), rtc.getMinutes(), rtc.getHours(),
                  rtc.getDate(), rtc.getMonth(), rtc.getYear());

    // Get the sensor RTC
    Serial.println(F("Getting sensor RTC."));
    int8_t  seconds = -1;
    int8_t  minutes = -1;
    int8_t  hours   = -1;
    int8_t  day     = -1;
    int8_t  month   = -1;
    int16_t year    = -1;
    sensor.getRTC(seconds, minutes, hours, day, month, year);
    Serial.print(F("    RTC: "));
    Serial.print(month);
    Serial.print(F("/"));
    Serial.print(day);
    Serial.print(F("/"));
    Serial.print(year);
    Serial.print(F(" "));
    Serial.print(hours);
    Serial.print(F(":"));
    Serial.print(minutes);
    Serial.print(F(":"));
    Serial.print(seconds);
    Serial.println();

    // Set Bulk Configuration
    // NOTE: This only works if there's a power cycle or reboot after writing
    // the configuration!
    Serial.print(F("Set sensor bulk configuration... "));
    // writeConfiguration(ANBSensorMode mode, ANBPowerStyle power,
    //                     ANBSalinityMode salinity, uint16_t delayHours,
    //                     uint16_t delayMinutes, uint16_t intervalHours,
    //                     uint16_t intervalMinutes, bool profilingEnabled,
    //                     bool modbusEnabled)
    bool bulkSet = sensor.writeConfiguration(
        ANBSensorMode::AUTONOMOUS, ANBPowerStyle::ALWAYS_POWERED, salinity,
        delayHours, delayMinutes, intervalHours, intervalMinutes,
        profilingEnabled, modbusEnabled);
    Serial.print(F(" ..."));
    Serial.println(bulkSet ? F("success") : F("failed"));

    // Now force the terminal interface to reboot the sensor
    modbusSerial.setTimeout(
        750);  // set a longer timeout for the terminal commands
    // clear anything hanging in the buffer
    do {
        Serial.println(modbusSerial.readString());
    } while (modbusSerial.available());
    sensor.forceTerminal();

    // enter the main menu of the terminal
    delay(15);  // short delay before the command
    modbusSerial.print("menu\r");
    modbusSerial.flush();
    Serial.println("menu");
    Serial.flush();
    // dump the response - we're not trying to parse the menu!
    uint32_t startTime = millis();
    while (millis() - startTime < 5000L && modbusSerial.available() < 10);
    do {
        Serial.println(modbusSerial.readString());
    } while (modbusSerial.available());
    // reboot the sensor for the setting to take
    delay(15);  // short delay before the command
    modbusSerial.print("reboot\r");
    modbusSerial.flush();
    Serial.println("reboot");
    Serial.flush();
    // dump the response
    startTime = millis();
    while (millis() - startTime < 10000L && modbusSerial.available() < 100);
    do {
        Serial.println(modbusSerial.readString());
    } while (modbusSerial.available());
    modbusSerial.setTimeout(1000L);  // restore the original timeout
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // listen to readings for forever
    // NOTE: Now that we're in autonomous mode, the sensor will only
    // take readings at the interval specified in the configuration and
    // will not respond to individual read requests.
    // When it does take readings, it will store them in its internal memory and
    // print them to the serial port.
    if (modbusSerial.available()) { Serial.println(modbusSerial.readString()); }
    if (millis() % 5000 == 0) {
        rtc.updateTime();
        Serial.println(rtc.stringTime8601());
        Serial.println(millis() / 1000);
        delay(1);
    }
}

// cspell: ignore DEREPin SWSERIAL spcond
