/** =========================================================================
 * @example{lineno} FindSensorBaud.ino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @license This example is published under the BSD-3 license.
 *
 * @brief This attempts to communicate with the sensor at all supported baud
 * rates.
 *
 * @m_examplenavigation{example_find_baud,}
 * @m_footernavigation
 * ======================================================================= */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
#include <Arduino.h>
#include <ANBSensorsModbus.h>

// ==========================================================================
//  Sensor Settings
// ==========================================================================

// Define the sensor's modbus address, or SlaveID
byte modbusAddress = 0x55;  // HEX 0x55 is the ANB default modbus address.

// The Modbus baud rate the sensor uses
uint32_t      modbusBaud = 57600;  // 57600 is ANB default baud rate.
ANBSensorBaud desiredBaud =
    ANBSensorBaud::BAUD57600;  // 57600 is ANB default baud rate.

// Sensor Timing
// Edit these to explore
#define WARM_UP_TIME 30000L      // milliseconds to wait for sensor to respond
#define STABILIZATION_TIME 100   // milliseconds for readings to stabilize.
#define MEASUREMENT_TIME 300000  // milliseconds to complete a measurement.

// #define TEST_POWER

// ==========================================================================
//  Data Logger Options
// ==========================================================================
const uint32_t serialBaud = 115200;  // Baud rate for serial monitor

// Define pin number variables
const int sensorPwrPin  = 56;  // The pin sending power to the sensor
const int adapterPwrPin = -1;  // The pin sending power to the RS485 adapter
const int DEREPin =
    -1;  // The pin controlling Receive Enable and Driver Enable
         // on the RS485 adapter, if applicable (else, -1)
         // Setting HIGH enables the driver (arduino) to send text
         // Setting LOW enables the receiver (sensor) to send text

// Turn on debugging outputs (i.e. raw Modbus requests & responses)
// by uncommenting next line (i.e. `#define DEBUG`)
#define DEBUG

// ==========================================================================
// Create and Assign a Serial Port for Modbus
// ==========================================================================
// Hardware serial ports are preferred when available.
#if defined(ANB_EXAMPLE_SERIAL_PORT)
#pragma message("Using user-defined serial port")
HardwareSerial& modbusSerial = ANB_EXAMPLE_SERIAL_PORT;

// AltSoftSerial is the most stable software (bit-banging) hardware serial
// alternative on AVR boards. Select over alternatives with the define below.
// #define BUILD_ALTSOFTSERIAL
#elif defined(BUILD_ALTSOFTSERIAL) && defined(__AVR__)
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
    SERIAL_PORT_USBVIRTUAL.begin(0);  // baud rate ignored
    while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000L)) { delay(100); }
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

    uint8_t       baudArrayIndex    = 0;
    ANBSensorBaud possible_bauds[8] = {
        ANBSensorBaud::BAUD57600, ANBSensorBaud::BAUD115200,
        ANBSensorBaud::BAUD9600,  ANBSensorBaud::BAUD14400,
        ANBSensorBaud::BAUD19200, ANBSensorBaud::BAUD28800,
        ANBSensorBaud::BAUD38400, ANBSensorBaud::BAUD56000,
    };
    uint32_t possible_baud_rates[8] = {57600, 115200, 9600,  14400,
                                       19200, 28800,  38400, 56000};

    bool communication_successful = false;


    while (!communication_successful && baudArrayIndex < 8) {
        Serial.print(F("\n\n\nTrying baud rate of "));
        Serial.println(possible_baud_rates[baudArrayIndex]);

        // Turn on your modbus serial port
#if defined(ESP8266)
        const int SSRxPin =
            13;  // Receive pin for software serial (Rx on RS485 adapter)
        const int SSTxPin =
            14;  // Send pin for software serial (Tx on RS485 adapter)
        modbusSerial.begin(possible_baud_rates[baudArrayIndex], SWSERIAL_8N1,
                           SSRxPin, SSTxPin, false);
#else  // For Hardware Serial
        modbusSerial.begin(possible_baud_rates[baudArrayIndex]);
#endif

        bool isReady = powerCycleSensor();

        // Try to force modbus if the enable failed
        if (!isReady) {
            Serial.println(F("Trying to get any modbus response... "));
            bool gotModbusResponse = sensor.gotModbusResponse();
            if (!gotModbusResponse) {
                Serial.println(
                    F("Did not get a modbus response, trying to force "
                      "Modbus enable... "));
                sensor.forceModbus();
                Serial.println(F("Trying again get a modbus response... "));
                gotModbusResponse = sensor.gotModbusResponse();
                Serial.print(F(" ..."));
                Serial.println(gotModbusResponse ? F("success") : F("failed"));
                if (gotModbusResponse) { isReady = true; }
            }
        }

        if (isReady) {
            Serial.print(F("Got Modbus response at "));
            Serial.print(possible_baud_rates[baudArrayIndex]);
            Serial.println(F("."));
            communication_successful = true;
        } else {
            baudArrayIndex++;
        }
    }

    if (!communication_successful) {
        Serial.println(F("Did not find the sensor at any baud rate!"));
        Serial.println(F("Use ANB Sensors Utility to confirm the sensor is in "
                         "Modbus mode."));
        Serial.println(F("Check wiring and modbus address and try again."));
        while (1);  // Stay here forevermore
    }

    if (possible_baud_rates[baudArrayIndex] != modbusBaud) {
        Serial.print(F("Setting sensor baud rate to desired rate of "));
        Serial.print(modbusBaud);
        Serial.println(F("... "));
        success = sensor.setBaud(desiredBaud);
        if (success) {
            Serial.println(F(" ...success"));
        } else {
            Serial.println(F(" ...failed"));
        }

        bool isReady = powerCycleSensor();
        if (!isReady) {
            Serial.println(
                F("Did not find the sensor again after changing baud rate."));
            Serial.println(
                F("Use ANB Sensors Utility to confirm the sensor is in "
                  "Modbus mode."));
            Serial.println(F("Check wiring and modbus address and try again."));
            while (1);  // Stay here forevermore
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
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    while (1);  // do nothing here
}

// cspell: ignore DEREPin SWSERIAL spcond
