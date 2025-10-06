/** =========================================================================
 * @example{lineno} DownloadData.ino
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

#include <SdFat.h>  // To communicate with the SD card
// The SD card and file
SdFat sd;
File  logFile;
#if defined(ARDUINO_AVR_ENVIRODIY_MAYFLY)
int8_t sdCardSSPin = 12;  // The slave select pin for the SD card
#elif defined(ENVIRODIY_STONEFLY_M4)
int8_t sdCardSSPin = 29;  // The slave select pin for the SD card
#elif defined(SDCARD_SS_PIN)
const int8_t sdCardSSPin = SDCARD_SS_PIN;
#else
const int8_t sdCardSSPin = 10;  // The slave select pin for the
#endif
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

// Turn on debugging outputs (i.e. raw Modbus requests & responses)
// by uncommenting next line (i.e. `#define DEBUG`)
#define DEBUG

// ==========================================================================
// Create and Assign a Serial Port for Modbus
// ==========================================================================
// Hardware serial ports are preferred when available.
// AltSoftSerial is the most stable alternative for modbus.
// Select over alternatives with the define below.
// #define BUILD_ALTSOFTSERIAL
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
uint32_t  startTime;
String    serialNumber     = "";
String    manufacturer     = "";
String    name             = "";
String    subname          = "";
String    interfaceVersion = "";
String    driverVersion    = "";

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

// This function is used to automatically mark files as
// created/accessed/modified when operations are done by the SdFat library. User
// provided date time callback function. See SdFile::dateTimeCallback() for
// usage.
void fileDateTimeCallback(uint16_t* date, uint16_t* time) {
    rtc.updateTime();
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(rtc.getYear(), rtc.getMonth(), rtc.getDate());
    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
}

// Protected helper function - This checks if the SD card is available and ready
bool initializeSDCard(void) {
    // If we don't know the slave select of the sd card, we can't use it
    if (sdCardSSPin < 0) {
        Serial.println(
            F("Slave/Chip select pin for SD card has not been set."));
        Serial.println(F("Data will not be saved!"));
        return false;
    }

    if (!sd.begin(sdCardSSPin)) {
        Serial.println(F("Error: SD card failed to initialize or is missing."));
        Serial.println(F("Data will not be saved!"));
        return false;
    } else {
        // skip everything else if there's no SD card, otherwise it might hang
        Serial.print(
            F("Successfully connected to SD Card with card/slave select on "
              "pin"));
        Serial.print(F(" "));
        Serial.println(sdCardSSPin);
        return true;
    }
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

    setSensorPower(false);
    delay(1000);
    setSensorPower(true);

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

    // Set a datetime callback for automatic time-stamping of files by SdFat
    SdFile::dateTimeCallback(fileDateTimeCallback);

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
    delay(5000L);  // Wait for everything to start up

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

    // Get Sensor Information
    Serial.print(F("\n\n\nGet sensor information...\n"));

    // Get the sensor serial number
    Serial.println(F("Getting sensor serial number."));
    serialNumber = sensor.getSerialNumber();
    Serial.print(F("    Serial Number: "));
    Serial.println(serialNumber);

    // Get the sensor manufacturer
    Serial.println(F("Getting sensor manufacturer."));
    manufacturer = sensor.getManufacturer();
    Serial.print(F("    Manufacturer: "));
    Serial.println(manufacturer);

    // Get the sensor name
    Serial.println(F("Getting sensor name."));
    name = sensor.getName();
    Serial.print(F("    Name: "));
    Serial.println(name);

    // Get the sensor sub-name
    Serial.println(F("Getting sensor sub-name."));
    subname = sensor.getSubName();
    Serial.print(F("    Subname: "));
    Serial.println(subname);

    // Get the sensor interface version
    Serial.println(F("Getting sensor interface version."));
    interfaceVersion = sensor.getInterfaceVersion();
    Serial.print(F("    Interface Version: "));
    Serial.println(interfaceVersion);

    // Get the sensor driver version
    Serial.println(F("Getting sensor driver version."));
    driverVersion = sensor.getDriverVersion();
    Serial.print(F("    Driver Version: "));
    Serial.println(driverVersion);

    // Set the sensor RTC
    Wire.begin();
    rtc.begin();
    rtc.updateTime();
    Serial.println(F("Setting sensor RTC."));
    sensor.setRTC(rtc.getSeconds(), rtc.getMinutes(), rtc.getHours(),
                  rtc.getDate(), rtc.getMonth(), rtc.getYear());

    // // Once we've got the metadata, have the sensor stay in the terminal
    // // interface - files can only be downloaded in terminal mode
    // Serial.println(F("Putting the sensor in terminal mode."));
    // sensor.enableTerminal();
    // sensor.forceReboot();

    // Initialise the SD card
    initializeSDCard();
}

uint16_t highestFileAttempted = 1;
uint16_t prevFileNum          = 0;
bool     lastFailed           = false;

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {
    // // dump anything in the serial buffer
    // do {
    //     Serial.println(Serial.readString());
    //     Serial.flush();
    //     delay(50);
    // } while (Serial.available());
    // Serial.println(F("Press any key to attempt a download..."));
    // // hang out until user presses a key
    // while (!Serial.available()) { delay(100); }
    // // read and discard the key
    // Serial.println("Got a key!");
    // do {
    //     Serial.println(Serial.readString());
    //     Serial.flush();
    //     delay(50);
    // } while (Serial.available());

    // // dump the sensor serial buffer
    // do {
    //     delay(100);
    //     modbusSerial.readStringUntil('\n');
    //     // Serial.println(modbusSerial.readStringUntil('\n'));
    // } while (modbusSerial.available());

    if (!lastFailed) { delay(1000); }

    if (lastFailed == true) {
        // Force the terminal interface to fetch all the data
        delay(15);  // short delay before the command
        modbusSerial.print("#700\r");
        modbusSerial.flush();
        Serial.println("#700");
        Serial.flush();
        uint32_t startTime = millis();
        while (millis() - startTime < 5000L && modbusSerial.available() < 10);
        bool gotForce = modbusSerial.available();
        if (!gotForce) {
            Serial.println(F("Did not get a response from the sensor!"));
            lastFailed = true;
            return;
        }
        do {
            delay(50);
            // modbusSerial.readString();
            Serial.println(modbusSerial.readString());
        } while (modbusSerial.available());
    }
    lastFailed = false;

    // enter the main menu of the terminal
    delay(15);  // short delay before the command
    modbusSerial.print("menu\r");
    modbusSerial.flush();
    Serial.println("menu");
    Serial.flush();
    // dump the response - we're not trying to parse the menu!
    startTime = millis();
    while (millis() - startTime < 2500L && modbusSerial.available() < 10);
    bool gotMenu = modbusSerial.available();
    if (!gotMenu) {
        Serial.println(F("Did not get a response from the sensor!"));
        lastFailed = true;
        return;
    }
    // dump the header of the response
    startTime           = millis();
    bool gotFailureLine = false;
    gotMenu             = false;
    while (!gotFailureLine && !gotMenu && millis() - startTime < 10000L) {
        String inLine = modbusSerial.readStringUntil('\n');
        Serial.println(inLine);
        if (inLine.indexOf("Please refer to the Menu") != -1) {
            gotFailureLine = true;
        }
        if (inLine.indexOf("Select result file(s) for download") != -1) {
            gotMenu = true;
        }
    }
    if (gotFailureLine) {
        lastFailed = false;
        return;
    }
    lastFailed = false;

    delay(15);  // short delay before the command
    modbusSerial.setTimeout(250L);
    modbusSerial.print("results\r");
    modbusSerial.flush();
    Serial.println("results");
    Serial.flush();
    // dump the header of the response
    startTime                = millis();
    bool gotEmptyWarningLine = false;
    while (!gotEmptyWarningLine && millis() - startTime < 10000L) {
        String inLine = modbusSerial.readStringUntil('\n');
        Serial.println(inLine);
        if (inLine.indexOf("Empty Files Are Not Listed") != -1) {
            gotEmptyWarningLine = true;
            break;
        }
    }

    // start parsing file names
    bool     isUnreadFile    = false;
    bool     shouldDownload  = false;
    uint16_t sensorFileCount = 1;
    String   dataFilename    = "";
    uint16_t fnum            = 0;
    String   ndTag           = "";
    String   sizeBytes       = "";
    String   sizeUnits       = "";
    String   fileDate        = "";
    startTime                = millis();
    do {
        String fileLine = modbusSerial.readStringUntil('\n');
        fileLine.trim();
        if (fileLine.length() < 12) { continue; }
        startTime = millis();

        // break if we hit the end of the directory listing
        if (fileLine.indexOf("**********END OF DIRECTORY**********") != -1) {
            break;
        }
        sensorFileCount++;

        dataFilename = fileLine.substring(0, 13);
        // Serial.print("File: ");
        // Serial.println(dataFilename);
        dataFilename.trim();
        fnum = dataFilename.substring(1, 8).toInt();
        // Serial.print("File Number: ");
        // Serial.println(fnum);
        ndTag = fileLine.substring(13, 14);
        // Serial.print("N/D Tag: ");
        // Serial.println(ndTag);
        sizeBytes = fileLine.substring(14, 23);
        // Serial.print("Size (KB): ");
        // Serial.println(sizeBytes);
        fileDate = fileLine.substring(27);
        // Serial.print("Date: ");
        // Serial.println(fileDate);

        Serial.print(fileLine);

        ndTag.trim();
        isUnreadFile = (ndTag != "D");  // N means New, D means downloaded
        // dataFilename.trim();
        shouldDownload = isUnreadFile;

        if (shouldDownload) {
            Serial.println(F("\t<-- Downloading "));
            break;
        } else {
            Serial.println(F("\t<xx Skipping"));
        }
    } while (!shouldDownload && millis() - startTime < 5000L);

    // dump the rest of the response
    bool gotEndDir = false;
    do {
        String fLine = modbusSerial.readStringUntil('\n');
        Serial.println(fLine);
        sensorFileCount++;
        if (fLine.indexOf("previously downloaded") != -1) { gotEndDir = true; }
    } while (millis() - startTime < 10000L && !gotEndDir);

    if (gotEndDir) {
        Serial.println(F("---------------------"));
        Serial.println(F("End of directory listing."));
        Serial.println(F("---------------------"));
        lastFailed = false;
    } else {
        Serial.println(F("\nDirectory listing incomplete!\n"));
        lastFailed = true;
        return;
    }

    Serial.print(F("\nThere are "));
    Serial.print(sensorFileCount);
    Serial.println(F(" files available for download."));
    Serial.flush();

    if (!shouldDownload) {
        Serial.println(F("No new files to download!"));
        Serial.flush();
        return;
    }
    highestFileAttempted++;


    // open the file if it exists
    initializeSDCard();
    dataFilename.trim();
    fileDate.trim();
    String useFileName = dataFilename;
    if (fileDate.length() == 8) {
        useFileName = dataFilename.substring(0, 8) + "_20" +
            fileDate.substring(6, 8) + fileDate.substring(0, 2) +
            fileDate.substring(3, 5) + dataFilename.substring(8);
    }
    if (serialNumber.length() > 1) {}
    { useFileName = serialNumber + "_" + useFileName; }
    if (logFile.open(useFileName.c_str(), O_WRITE | O_AT_END)) {
        Serial.print(F("File: "));
        Serial.print(useFileName);
        Serial.println(F(" already exists. Appending data."));
        Serial.flush();
    } else if (logFile.open(useFileName.c_str(),
                            O_CREAT | O_WRITE | O_AT_END)) {
        Serial.print(F("Created new file:"));
        Serial.println(useFileName);
        Serial.flush();
    } else {
        Serial.print(F("Could not create file: "));
        Serial.println(useFileName);
        Serial.println(F("Data will not be saved!"));
        Serial.flush();
        return;
    }

    // Add a header to the file (it has none!)
    if (serialNumber.length() > 1) {
        logFile.print(F("System Manufacturer        : "));
        logFile.println(manufacturer);
        logFile.print(F("System Type                : "));
        logFile.println(name);
        logFile.print(F("Sub System Type            : "));
        logFile.println(subname);
        logFile.print(F("Interface Firmware Version : "));
        logFile.println(interfaceVersion);
        logFile.print(F("Driver Firmware Version    : "));
        logFile.println(driverVersion);
        logFile.print(F("System Serial Number       : "));
        logFile.println(serialNumber);
        logFile.println();
    }
    logFile.print(F("TIMESTAMP,PH,TEMPERATURE,SALINITY,CONDUCTIVITY,TRANSDUCER_"
                    "HEALTH,SENSOR_DIAGNOSTICS,RESERVED_01,RESERVED_02"));


    // request the file
    modbusSerial.print(String(fnum) + "\r");
    modbusSerial.flush();
    Serial.println(String(fnum));
    Serial.flush();

    // write everything to the SD card
    startTime = millis();
    while (millis() - startTime < 10000L && modbusSerial.available() < 12);
    // toss the first four lines
    for (int i = 0; i < 4; i++) {
        Serial.print(modbusSerial.readStringUntil('\n'));
    }
    Serial.println(F("---"));
    bool gotTail = false;
    startTime    = millis();
    do {
        String reading = modbusSerial.readStringUntil('\n');
        if (reading.length() < 12) { continue; }
        // bump time for line if it's more than 12 characters
        startTime = millis();
        Serial.println(reading);
        Serial.flush();

        if (reading.indexOf("**********END OF FILE**********") != -1) {
            gotTail = true;
            break;  // break so the "END OF FILE" line is not written
        }

        logFile.print(reading);
        logFile.flush();
    } while (millis() - startTime < 10000L && !gotTail);
    // Close the file to save it
    logFile.close();

    if (gotTail) {
        Serial.println(F("---------------------"));
        Serial.println(F("File download complete!"));
        Serial.println(F("---------------------"));
        Serial.flush();
        prevFileNum = fnum;
    } else {
        Serial.println(F("File download incomplete!"));
    }

    // throw out last bit of the response
    do {
        delay(50);
        // modbusSerial.readStringUntil('\n');
        Serial.println(modbusSerial.readStringUntil('\n'));
    } while (modbusSerial.available());
}

// cspell: ignore DEREPin SWSERIAL spcond
