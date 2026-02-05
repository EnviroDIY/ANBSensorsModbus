/**
 * @file ANBSensorsModbus.h
 * Part of the EnviroDIY ANBSensorsModbus library for Arduino.
 * @license This library is published under the BSD-3 license.
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the ANBSensorsModbus class declarations.
 *
 * Tested with AQ5
 */

#ifndef ANBSensorsModbus_h
#define ANBSensorsModbus_h

#include <Arduino.h>
#include <SensorModbusMaster.h>
#include <fast_math.h>  // For BCD conversions

/**
 * @brief Default Modbus address for ANB sensors.
 *
 * This is the default address used for Modbus communication with ANB sensors.
 */
#define ANB_DEFAULT_MODBUS_ADDRESS 0x55

/**
 * @brief Sensor control modes for ANB sensors.
 *
 * CONTROLLED mode means the sensor is controlled by an external system.
 * AUTONOMOUS mode means the sensor operates independently.
 *
 * @note In controlled mode, the sensor remains continuously connected
 * throughout deployment allowing you to control it directly at all times. In
 * autonomous mode, the sensor is configured before deployment, and once
 * deployed, operates independently.
 *
 * [Details about the autonomous mode are available
 * here.](https://www.anbsensors.com/newdocs/docs/Sensor%20Controls%20&%20Functions/autonomous%20monitoring)
 */
enum class ANBSensorMode {
    CONTROLLED = 1,  ///< use controlled mode - measurement taken when demanded
                     ///< by an external controller
    AUTONOMOUS = 2,  ///< use autonomous mode - sensor operates independently
                     ///< following a pre-set routine
    UNKNOWN = 255    ///< Unknown sensor mode; no response from the sensor
};

/**
 * @brief Salinity modes for the ANB sensors.
 *
 * These modes configure the sensor for different salinity ranges.
 */
enum class ANBSalinityMode : uint8_t {
    LOW_SALINITY  = 1,   ///< use low salinity mode - (0.05-2.5ppt)
    HIGH_SALINITY = 2,   ///< use high salinity mode - (2.5-40ppt)
    UNKNOWN       = 255  ///< Unknown salinity mode; no response from the sensor
};

/**
 * @brief Power styles for the ANB sensors.
 *
 * This tells the sensor how it will be powered.  It does NOT control the
 * power state of the sensor; it only informs the sensor of the expected
 * power management strategy.
 */
enum class ANBPowerStyle : uint8_t {
    ALWAYS_POWERED = 1,  ///< The sensor will be continuously powered
    ON_MEASUREMENT =
        2,         ///< The sensor will be powered only during measurements
    UNKNOWN = 255  ///< Unknown power style; no response from the sensor
};
/**
 * @brief Baud rates for the ANB sensors.
 *
 * These are the valid baud rates for communication with the sensor.
 * The default baud rate is 57600.
 * All data is sent in 8-N-1 format (8 data bits, no parity, 1 stop bit).
 * The parity cannot be changed.
 *
 * [Detailed communication settings are available
 * here.](https://www.anbsensors.com/newdocs/docs/modbus#sensor-communication)
 */
enum class ANBSensorBaud : uint8_t {
    BAUD9600   = 1,   ///< 9600 baud
    BAUD14400  = 2,   ///< 14400 baud
    BAUD19200  = 3,   ///< 19200 baud
    BAUD28800  = 4,   ///< 28800 baud
    BAUD38400  = 5,   ///< 38400 baud
    BAUD56000  = 6,   ///< 56000 baud
    BAUD57600  = 7,   ///< 57600 baud
    BAUD115200 = 8,   ///< 115200 baud
    UNKNOWN    = 255  ///< Unknown baud rate; no response from the sensor
};

// clang-format off
/**
 * @brief Health codes for the ANB sensors.
 *
 * These codes indicate the health status of the sensor's transducer.
 *
 * | ANBHealthCode | Value | Explanation                            | Action |
 * | ------------- | ----- | -------------------------------------- |------- |
 * | OK            | 0     | Healthy Transducer                     | No action required |
 * | ABRADE_SOON   | 1     | Transducer will need abrading soon     | - Ensure salinity setting is correct<br>- Abrade the transducer based on scanning profile and access frequency;<br>if accessed daily, wait until the health number reaches 2 before abrading,<br>otherwise, abrade now for sensors not accessed for weeks/months |
 * | ABRADE_NOW    | 2     | Transducer needs abrading now          | - Ensure salinity setting is correct<br>- Abrade the transducer |
 * | REPLACE       | 3     | Transducer needs replacing             | - Ensure salinity setting is correct<br>- Replace the transducer (or transducer not immersed for sensors with serial numbers less than 300200) |
 * | NOT_IMMERSED  | 4     | Transducer is not immersed             | Immerse the sensor |
 * | NO_REFERENCE  | 5     | No valid reference tracker measurement | Please wait for the next measurement |
 * | NO_PH         | 6     | No valid pH measurement                | Please wait for the next measurement |
 * | UNKNOWN       | 255   | Unknown status                         | Reconnect with the sensor |
 *
 * [Transducer health details can be found here.](https://www.anbsensors.com/newdocs/docs/sensor-output#transducer-health)
 *
 * [Detailed maintenance guidelines including how to abrade the sensor can be found here.](https://www.anbsensors.com/newdocs/docs/transducer-maintenance/)
 */
enum class ANBHealthCode:uint8_t {
    OK           = 0,   ///< Healthy Transducer; No action required
    ABRADE_SOON  = 1,   ///< Transducer will need abrading soon
    ABRADE_NOW   = 2,   ///< Transducer needs abrading now
    REPLACE      = 3,   ///< Transducer needs replacing
    NOT_IMMERSED = 4,   ///< Transducer is not immersed
    NO_REFERENCE = 5,   ///< No valid reference tracker measurement
    NO_PH        = 6,   ///< No valid pH measurement
    UNKNOWN      = 255  ///< Unknown health status; no response from the sensor
};
// clang-format on

// clang-format off
/**
 * @brief Status codes for the ANB sensors.
 *
 * These codes indicate the current operational status of the sensor.
 *
 * | ANBStatusCode       | Value | Explanation                                                     |
 * | ------------------- | ----- | --------------------------------------------------------------- |
 * | SLEEPING            | 0     | Sensor is idle and/or following its pre-set interval routine    |
 * | INTERVAL_SCANNING   | 1     | pH will be updated when measurement completes                   |
 * | CONTINUOUS_SCANNING | 2     | pH will be automatically updated as per our continuous sequence |
 * | UNKNOWN             | 255   | Unknown; no response from the sensor                     |
 *
 * [Sensor status code documentation is available here.](https://www.anbsensors.com/newdocs/docs/modbus#sensor-diagnostics)
 */
enum class ANBStatusCode:uint8_t {
    SLEEPING            = 0,   ///< Sleep
    INTERVAL_SCANNING   = 1,   ///< Interval Scanning
    CONTINUOUS_SCANNING = 2,   ///< Continuous Scanning
    UNKNOWN             = 255  ///< Unknown; no response from the sensor
};
// clang-format on

// clang-format off
/**
 * @brief Diagnostics codes for the ANB sensors.
 *
 * | ANBDiagnosticCode | Value | Explanation         | Action |
 * | ----------------- | ----- | ------------------- | ------ |
 * | OK                | 0     | Healthy Sensor      | None |
 * | BATTERY_ERROR     | 1     | Clock Battery Error | If there is no external power to the sensor the real time clock will not hold the programmed time<br>- If the sensor is powered, the time set and data streamed, this failure is not an issue<br>- If the sensor is in autonomous mode the clock will fail if disconnected from the power when it was programmed and placed on an external battery<br>-Users can record when the sensor was first switched on (where the sensor's time will be set to 0) and calculate the times externally, or, if this is not viable, contact support@anbsensors.com |
 * | SD_ERROR          | 2     | SD Card Error       | Either SD Card has been disabled or<br>A failing in the internal data save has occurred and no new data can be saved to the internal memory<br>- If the sensor is connected to an external communications system the sensor will continue to stream data, however no data will be saved in autonomous mode<br>- Please contact support@anbsensors.com |
 * | SYSTEM_ERROR      | 3     | System Error        | Contact support@anbsensors.com |
 * | UNKNOWN           | 255   | Unknown status      | Reconnect with the sensor |
 *
 * [Sensor diagnostic output details can be found here.](https://www.anbsensors.com/newdocs/docs/sensor-output#sensor-diagnostics)
 */
enum class ANBDiagnosticCode:uint8_t {
    OK            = 0,  ///< Healthy Sensor; No action required
    BATTERY_ERROR = 1,  ///< Clock Battery Error
    SD_ERROR      = 2,  ///< SD Card Error
    SYSTEM_ERROR  = 3,  ///< System Error
    UNKNOWN = 255  ///< Unknown; no response from the sensor
};
// clang-format on

/**
 * @brief The Main Class
 *
 * This is the class for communication with ANB pH sensors via modbus.
 */
class anbSensor {

 public:

    /**
     * @anchor ctor_and_begin
     * @name Constructors and Begins
     *
     * Functions to create the anbSensor object and set up the communication
     * with the Arduino stream connected to the modbus device.
     */
    /**@{*/
    /**
     * @brief Default constructor
     */
    anbSensor();
    /**
     * @brief Construct a new anbSensor object
     *
     * @see setAddress(byte)
     *
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A pointer to the Arduino stream object to communicate with.
     */
    anbSensor(byte modbusSlaveID, Stream* stream);
    /**
     * @brief Construct a new anbSensor object
     *
     * @see setAddress(byte)
     *
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A reference to the Arduino stream object to communicate
     * with.
     */
    anbSensor(byte modbusSlaveID, Stream& stream);
    /**
     * @copydoc anbSensor(byte modbusSlaveID, Stream* stream)
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply.
     */
    anbSensor(byte modbusSlaveID, Stream* stream, int8_t enablePin);
    /**
     * @copydoc anbSensor(byte modbusSlaveID, Stream& stream)
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply.
     */
    anbSensor(byte modbusSlaveID, Stream& stream, int8_t enablePin);
    /**
     * @brief Construct a new anbSensor object
     *
     * @param stream A pointer to the Arduino stream object to communicate with.
     */
    anbSensor(Stream* stream);
    /**
     * @brief Construct a new anbSensor object
     *
     * @param stream A reference to the Arduino stream object to communicate
     * with.
     */
    anbSensor(Stream& stream);
    /**
     * @copydoc anbSensor(Stream* stream)
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply.
     */
    anbSensor(Stream* stream, int8_t enablePin);
    /**
     * @copydoc anbSensor(Stream& stream)
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply.
     */
    anbSensor(Stream& stream, int8_t enablePin);

    /**
     * @brief Equivalent to a constructor - used to assign members of the
     * anbSensor object
     *
     * @see setAddress(byte)
     *
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A pointer to the Arduino stream object to communicate with.
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply. Optional with a default value of -1.
     * @return Always returns true
     */
    bool begin(byte modbusSlaveID, Stream* stream, int enablePin = -1);
    /**
     * @brief Equivalent to a constructor - used to assign members of the
     * anbSensor object
     *
     * @see setAddress(byte)
     *
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param stream A reference to the Arduino stream object to communicate
     * with.
     * @param enablePin A pin on the Arduino processor to use to send an enable
     * signal to an RS485 to TTL adapter. Use a negative number if this does not
     * apply. Optional with a default value of -1.
     * @return Always returns true
     */
    bool begin(byte modbusSlaveID, Stream& stream, int enablePin = -1);
    /**
     * @brief Equivalent to a constructor - used to assign members of the
     * anbSensor object
     *
     * @param modbusSlaveID The byte identifier of the modbus slave device.
     * @param modbus A reference to the modbusMaster object to communicate
     * with.
     * @return Always returns true
     */
    bool begin(byte modbusSlaveID, modbusMaster& modbus);
    /**@}*/

    /**
     * @anchor setters_and_getters
     * @name Object setters and getters
     *
     * Functions to set and get properties of the anbSensor object.
     */
    /**@{*/

    /**
     * @brief Get the Last Error object
     * @return The last Modbus error code.
     */
    modbusErrorCode getLastError() {
        return modbus.getLastError();
    }
    /**@}*/


    /**
     * @anchor comm_timing_fxns
     * @name Functions to verify communication and check for sensor readiness
     *
     * These functions allow you to check if the sensor is ready, if a
     * measurement is complete, and if the sensor has responded to the last
     * Modbus command.
     */
    /**@{*/
    /**
     * @brief Check if the sensor has responded to the last Modbus command.
     *
     * @warning This function only checks for a response to the last Modbus
     * command. A correctly formatted Modbus error response is considered a
     * valid response for this function.
     *
     * @return True if the sensor has responded, false if not.
     */
    bool gotModbusResponse(void);

    /**
     * @brief Check if the sensor is ready for a command or has completed a
     * measurement.
     *
     * @return True if the sensor is ready, false if not.
     */
    bool isSensorReady(void);

    /**
     * @brief Check if a measurement is complete
     *
     * > **First Scan Command Delay**
     * >
     * > After sending the first scan command there is a 2-3 min delay before
     * > the sensor will return a valid pH value. In this 90s the sensor will
     * > output an Exception which is signified by setting the MSB in the
     * > command byte and adding a payload byte that specifies the reason for
     * > the exception.
     * >
     * > i.e. for the request:
     * >
     * > `< ADDRESS >< 03 >< 00 >< 00 >< 00 >< 02 >< CRC >`
     * >
     * > The exception reply when there is currently not a pH value to return
     * > is:
     *
     * > `< ADDRESS >< 83 >< 05 >< CRC >`
     *
     * > Where 05 = ACKNOWLEDGE This exception signifies that the sensor has
     * > received the Modbus communication but cannot currently perform the
     * > requested function. For example, requesting the current pH value when
     * > the sensor is not scanning.
     * >
     * > or
     * >
     * > `< ADDRESS >< 83 >< 06 >< CRC >`
     * >
     * > Where 06 = BUSY This exception is transmitted when the sensor needs
     * > more time before it can perform the requested function. It will be
     * > necessary to re-transmit the request again at a later time. For
     * > example, requesting the current pH value, before the sensor has been
     * > able to calculate the value.
     *
     * The error responses documented by ANB Sensors will be returned for
     * requests for pH values or (in firmware prior to
     * IB 10.11.11/STM 10.11.73D) health codes, but **not** when requesting the
     * status code.  The request for the status code (and the health code in
     * firmware >= IB 10.11.11/STM 10.11.73D) will return a valid response even
     * before the sensor is ready to start scanning.
     *
     * @return True if the measurement is complete, false if not.
     */
    bool isMeasurementComplete(void);
    /**@}*/


    /**
     * @anchor measurement_setting_fxns
     * @name Functions to configure measurements
     *
     * These functions allow you to set mode, set salinity, set interval time,
     * etc.
     *
     * [Detailed function documentation is available
     * here.](https://www.anbsensors.com/newdocs/docs/modbus/#measurement-setting-functions)
     *
     * @note The Modbus instructions from ANB sensors state that the "source"
     * for all of the measurement setting functions are input (read only)
     * registers, but in all cases the functions use **holding** (read/write)
     * registers.  Most of the registers are, however, **write only** from
     * within the modbus interface - attempting to read the registers will give
     * an illegal function error.
     */
    /**@{*/

    /**
     * @brief Get the sensor control mode
     *
     * The control mode is in ~~input~~ **holding** register 0x0035 (decimal
     * 53).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The sensor control mode is
     * **write only** from within the modbus interface.
     *
     * @see ANBSensorMode
     *
     * @return A code for the control mode
     */
    ANBSensorMode getControlMode(void)
        __attribute__((error("Command not available!")));
    /**
     * @brief Set the sensor control mode
     *
     * Change this value to any of the following valid values:
     * ANBSensorMode::CONTROLLED, ANBSensorMode::AUTONOMOUS
     *
     * @note Requires power cycle to invoke autonomous mode
     *
     * The control mode is in ~~input~~ **holding** register 0x0035 (decimal
     * 53).
     *
     * @see ANBSensorMode
     *
     * @param newControlMode The new control mode to use
     * @return True if the control mode was successfully set, false if not.
     */
    bool setControlMode(ANBSensorMode newControlMode);

    /**
     * @brief Get the sensor salinity mode
     *
     * The salinity mode is in ~~input~~ **holding** register 0x003E (decimal
     * 62).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The salinity mode is
     * **write only** from within the modbus interface.
     *
     * @return A code for the salinity mode
     */
    ANBSalinityMode getSalinityMode(void)
        __attribute__((error("Command not available!")));
    /**
     * @brief Set the sensor salinity mode
     *
     * Change this value to any of the following valid values:
     * ANBSalinityMode::LOW_SALINITY, ANBSalinityMode::HIGH_SALINITY
     *
     * @note Before scanning set the expected salinity.
     *
     * The salinity mode is in ~~input~~ **holding** register 0x003E (decimal
     * 62).
     *
     * @param newSalinityMode The new salinity mode to use
     * @return True if the salinity mode was successfully set, false if not.
     */
    bool setSalinityMode(ANBSalinityMode newSalinityMode);

    /**
     * @brief Get the sensor power style
     *
     * The power style is in ~~input~~ **holding** register 0x003F (decimal 63).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The power style is **write
     * only** from within the modbus interface.
     *
     * @return A code for the power style
     */
    ANBPowerStyle getPowerStyle(void)
        __attribute__((error("Command not available!")));
    /**
     * @brief Set the sensor power style
     *
     * Change this value to any of the following valid values:
     * ANBPowerStyle::ALWAYS_POWERED, ANBPowerStyle::ON_MEASUREMENT
     *
     * @note Before scanning set the desired power style.
     *
     * The power style is in ~~input~~ **holding** register 0x003F (decimal 63).
     *
     * @param newPowerStyle The new power style to use
     * @return True if the power style was successfully set, false if not.
     */
    bool setPowerStyle(ANBPowerStyle newPowerStyle);

    /**
     * @brief Get the current interval time
     *
     * Returns the interval time in minutes. A value of 0 indicates continuous
     * operation, while any other value represents the interval in minutes
     * between measurements. The minimum interval time for interval mode is 10
     * minutes. The maximum interval time is 240 minutes.
     *
     * The interval time is stored in ~~input~~ **holding** register 0x0036
     * (decimal 54).
     *
     * @note The interval time only applies if the sensor is in autonomous mode
     * and always powered.  The sensor will go into a low power state during the
     * set interval.
     *
     * @return The current interval time in minutes. 0 indicates continuous
     * operation.
     */
    uint8_t getIntervalTime(void);
    /**
     * @brief Set the sensor interval time
     *
     * Change this value to any of the following valid values:
     * - 0 for continuous operation
     * - 10-240 for interval mode (in minutes)
     *
     * The new interval time **does not take effect** until the next bootup.
     *
     * @note The interval time only applies if the sensor is in autonomous mode
     * and always powered.  The sensor will go into a low power state during the
     * set interval.
     *
     * The interval time is stored in ~~input~~ **holding** register 0x0036
     * (decimal 54).
     *
     * @param newIntervalTime The new interval time to use
     * @return True if the interval time was successfully set, false if not.
     */
    bool setIntervalTime(uint8_t newIntervalTime);

    /**
     * @brief Get the current sensor start delay
     *
     * The start delay is the time that the sensor waits before taking its first
     * measurement after booting up. A value of 0 indicates no delay. The
     * maximum value is 24 hours.
     *
     * @note The start delay only applies if the sensor is in autonomous mode
     * and always powered.
     *
     * The start delay is in ~~input~~ **holding** registers 0x0042-0x0043
     * (decimal 66-67).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The start delay is **write
     * only** from within the modbus interface.
     *
     * @return True if the start delay was successfully retrieved, false if not.
     * @param hours Reference to a uint16_t where the hours part of the start
     * delay will be stored.
     * @param minutes Reference to a uint16_t where the minutes part of the
     * start delay will be stored.
     */
    bool getStartDelay(uint16_t& hours, uint16_t& minutes)
        __attribute__((error("Command not available!")));
    /**
     * @brief Set the sensor start delay
     *
     * The start delay is the time that the sensor waits before taking its first
     * measurement after booting up. A value of 0 indicates no delay. The
     * maximum value is 24 hours.
     *
     * @note The start delay only applies if the sensor is in autonomous mode
     * and always powered.
     *
     * The start delay is in ~~input~~ **holding** registers 0x0042-0x0043
     * (decimal 66-67).
     *
     * @param delayHours The hours part of the new start delay (0-24)
     * @param delayMinutes The minutes part of the new start delay (0-59)
     * @return True if the start delay was successfully set, false if not.
     */
    bool setStartDelay(uint16_t delayHours, uint16_t delayMinutes);

    /**
     * @brief Check if the immersion sensor is enabled
     *
     * > - The immersion sensor checks whether the sensor is submerged in water.
     * > - If the sensor is not immersed, it will stop measurement and set the
     * >   Transducer Health status to 4.
     * > - In Continuous Mode, the sensor will wait for 5 minutes before
     * >   checking again.
     * > - In Interval Delay Mode, the sensor will wait for the configured
     * >   interval delay before rechecking.
     * > - Measurement will automatically resume once immersion is detected.
     *
     * @note This function is not available on ATX sensors (S/N <300200).
     *
     * The immersion sensor status (immersion rule) is in ~~input~~ **holding**
     * register 0x003C (decimal 60).
     *
     * @return True if the immersion sensor is enabled, false otherwise.
     */
    bool isImmersionSensorEnabled(void);
    /**
     * @brief Enable or disable the immersion sensor
     *
     * @note The new immersion sensor status (immersion rule) is effective
     * immediately.  When power cycled, the immersion sensor defaults to enabled
     * and the sensor goes into a low power mode.
     *
     * The immersion sensor status (immersion rule) is in ~~input~~ **holding**
     * register 0x003C (decimal 60).
     *
     * @param enable True to enable the immersion sensor, false to disable
     * @return True if the immersion sensor status was successfully set, false
     * if not.
     */
    bool enableImmersionSensor(bool enable = true);

    /**
     * @brief Check if fast profiling is enabled
     *
     * @return True if fast profiling is enabled, false otherwise.
     *
     * @note Fast profiling allows for a faster response to pH changes, with an
     * accuracy of +/- 0.1 pH unit.  Fast profiling is only available when
     * taking continuous measurements - which also requires that the sensor be
     * in autonomous mode and always powered.
     *
     * The profiling mode is in ~~input~~ **holding** register 0x0041 (decimal
     * 65).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The profiling mode is
     * **write only** from within the modbus interface.
     */
    bool isFastProfilingEnabled(void)
        __attribute__((error("Command not available!")));
    /**
     * @brief Enables or disables the fast profiling mode
     *
     * @param enable True to enable fast profiling mode, false to disable
     * @return True if the command was successfully sent, false if not.
     */
    bool enableFastProfiling(bool enable = true);

    /**
     * @brief Check if SD card is enabled
     *
     * @return True if SD card is enabled, false otherwise.
     *
     * @note The sensor has internal memory which can be switched off to save
     * power if necessary. If the internal memory is switched off, no data will
     * be saved in the sensor. All data must be saved externally.
     *
     * @note Data can only be downloaded the you computer of uploaded to the ANB
     * cloud using the serial terminal interface or ANB utils.
     *
     * The SD card status is in ~~input~~ **holding** register 0x0040 (decimal
     * 64).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The SD card status is
     * **write only** from within the modbus interface.
     */
    bool isSDCardEnabled(void) __attribute__((error("Command not available!")));
    /**
     * @brief Enables or disables the SD card
     *
     * @note Disabling the SD card will reduce power consumption by the sensor.
     *
     * @param enable True to enable SD card, false to disable
     * @return True if the command was successfully sent, false if not.
     */
    bool enableSDCard(bool enable = true);

    /**
     * @brief Write a bulk configuration to the sensor
     *
     * @see The $UTIL string in the [documentation for configuring multiple
     * sensors](https://www.anbsensors.com/newdocs/docs/Sensor%20Controls%20&%20Functions/configuring%20multiple%20sensors)
     *
     * @param mode The sensor control mode
     * @param power The power style
     * @param salinity The salinity mode
     * @param delayHours The hour portion of the delay before the first sample
     * @param delayMinutes The minute portion of the delay before the first
     * sample
     * @param intervalHours The hour portion of the interval between samples
     * @param intervalMinutes The minute portion of the interval between samples
     * @param profilingEnabled True to enable fast profiling, false to disable
     * @param modbusEnabled True to enable Modbus, false to disable
     * @return True if the configuration was successfully written, false if not.
     *
     * @warning You must reboot or power cycle the sensor after sending this
     * command!  No other commands will be accepted until the sensor is rebooted
     * or power cycled.
     */
    bool writeConfiguration(ANBSensorMode mode, ANBPowerStyle power,
                            ANBSalinityMode salinity, uint16_t delayHours,
                            uint16_t delayMinutes, uint16_t intervalHours,
                            uint16_t intervalMinutes, bool profilingEnabled,
                            bool modbusEnabled);
    /**@}*/


    /**
     * @anchor command_fxns
     * @name Functions to start and stop measurements
     *
     * [Detailed function documentation is available
     * here.](https://www.anbsensors.com/newdocs/docs/modbus/#command-functions)
     */
    /**@{*/

    /**
     * @brief Tells a read-only sensor to begin a scan (taking measurements)
     *
     * A scan is starting by sending the command {ADDR 03 00 AA 00 0B CRCL CRCH}
     * after which the sensor will respond with {ADDR 03 04 00 00 00 00 CRCL
     * CRCH} if successful.
     *
     * @warning This function is only for use with read-only sensors.  This
     * command is **not** a valid modbus command/response pattern!  The command
     * 0x03 should be used to read from a holding register and the response
     * should include the number of registers requested. In this command we are
     * requesting 0x000B (11) registers but the response indicates 0x04 (4)
     * bytes, which only corresponds to 2 registers.
     *
     * @return True if the scan was successfully started, false if not.
     */
    bool startReadOnly(void);

    /**
     * @brief Tells the sensor to begin a scan (taking measurements)
     *
     * The start scan command is set with **coil** 0x0100 (decimal 256).
     *
     * > After sending the first scan command there is a 2-3 min delay before
     * > the sensor will return a valid pH value. In this 90s the sensor will
     * > output an Exception which is signified by setting the MSB in the
     * > command byte and adding a payload byte that specifies the reason for
     * > the exception.
     * >
     * > i.e. for the request:
     * >
     * > < ADDRESS >< 03 >< 00 >< 00 >< 00 >< 02 >< CRC >
     * >
     * > The exception reply when there is currently not a pH value to return
     * > is:
     * >
     * > < ADDRESS >< 83 >< 05 >< CRC >
     * >
     * > Where 05 = ACKNOWLEDGE This exception signifies that the sensor has
     * > received the Modbus communication but cannot currently perform the
     * > requested function. For example, requesting the current pH value when
     * > the sensor is not scanning.
     * >
     * > or
     * >
     * > < ADDRESS >< 83 >< 06 >< CRC >
     * >
     * > Where 06 = BUSY This exception is transmitted when the sensor needs
     * > more time before it can perform the requested function. It will be
     * > necessary to re-transmit the request again at a later time. For
     * > example, requesting the current pH value before the sensor has been
     * > able to calculate the value.
     * >
     * > NB. All data is big endian
     *
     * @warning Do not run the sensors in pH buffer solutions. They are for use
     * in seawater - freshwater.
     *
     * @return True if the scan was successfully started, false if not.
     */
    bool start(void);

    /**
     * @brief Inform the sensor that the transducer has been abraded
     *
     * If the sensor is told that it has been abraded, it resets all internal
     * sensor settings.
     *
     * The abrade sensor command is set with **coil** 0x0180 (decimal 384).
     *
     * [Detailed maintenance guidelines including how to abrade the sensor can
     * be found
     * here.](https://www.anbsensors.com/newdocs/docs/transducer-maintenance/)
     *
     * @return True if the sensor was successfully informed of the abrasion,
     * false if not.
     */
    bool abradeSensor(void);

    /**
     * @brief Tells the sensors to stop scanning (taking measurements)
     *
     * This saves the current results and closes the pH results file, but leaves
     * the sensor powered.  The next start-measurement command will open a new
     * file, if the SD card is enabled.
     *
     * The stop command is set with **coil** 0x0000
     *
     * @return True if the scan was successfully stopped, false if not.
     */
    bool stop(void);

    /**
     * @brief Tells the sensors to save the current sensor configuration and
     * restart the sensor so it is ready to run with the saved configurations.
     *
     * The reboot command is set by writing 0xFFFF to ~~input~~ **holding**
     * register 0x1000
     *
     * @warning Due to errors in firmware 10.10, after sending this command, the
     * sensor may revert to terminal mode, requiring a `forceModbus()` command
     * to put it back into modbus mode.  This is expected to be fixed in the
     * next pH sensor firmware release.
     *
     * @return True if the sensor was successfully rebooted and began responding
     * to Modbus commands again, false if not.
     */
    bool reboot(void);

    /**
     * @brief Force the sensor to reboot using terminal mode commands
     * @param alreadyInTerminal If true, the sensor is already in terminal mode;
     * if false (default) the sensor will be sent the command to enter terminal
     * mode first.
     * @return True if the sensor was successfully rebooted and began responding
     * to Modbus commands again, false if not.
     */
    bool forceReboot(bool alreadyInTerminal = false);

    /**
     * @brief Tells the sensors to save the current results and turn off the
     * sensor.
     *
     * This saves the current results and closes the pH results file leaving the
     * sensor ready to power down.
     *
     * The reboot command is set by writing 0x0000 to ~~input~~ **holding**
     * register 0x0200 (decimal 512).
     *
     * @warning After sending a shutdown command, the sensor **MUST BE POWER
     * CYCLED** to resume sampling
     *
     * @return True if the sensor was successfully rebooted, false if not.
     */
    bool shutdown(void);
    /**@}*/


    /**
     * @anchor measurement_output_fxns
     * @name Functions to get one or more values from a sensor.
     *
     * [Detailed function documentation is available
     * here.](https://www.anbsensors.com/newdocs/docs/modbus/#measurement-output-functions)
     */
    /**@{*/

    /**
     * @brief Gets the current pH value from the sensor.
     *
     * @note If the pH output is 99.99, check the transducer health number for
     * instruction.
     *
     * The pH value is stored in holding register 0x0000 (decimal 0).
     *
     * @return The pH value as a float. Passes the sensor returned 99.99 for a
     * bad value and returns -9999 if the sensor could not be read.
     */
    float getpH(void);

    /**
     * @brief Gets the current temperature in degrees Celsius (°C) from the
     * sensor.
     *
     * The temperature value is stored in holding register 0x0002 (decimal 2).
     *
     * @return The temperature value in degrees Celsius (°C) as a float. Passes
     * the sensor returned 99.99 for a bad value and returns -9999 if the sensor
     * could not be read.
     */
    float getTemperature(void);

    /**
     * @brief Gets the current salinity in parts per thousand (ppt) from the
     * sensor.
     *
     * @note If the salinity output is 99.99 but the pH output is
     * OK, the salinity is out of range.
     * - Try changing your salinity setting
     * - If expected salinity is > 7ppt no salinity output is given
     *
     * @note If both the pH and salinity output is 99.99, check the
     * transducer health number for instruction.
     *
     * The salinity value is stored in holding register 0x0004 (decimal 4).
     *
     * @return The salinity value as a float. Passes the sensor
     * returned 99.99 for a bad value and returns -9999 if the sensor could not
     * be read.
     */
    float getSalinity(void);

    /**
     * @brief Gets the current specific conductance in millisiemens per
     * centimeter (mS/cm) from the sensor.
     *
     * @note If the specific conductance output is 99.99 but the pH output is
     * OK, the salinity is out of range.
     * - Try changing your salinity setting
     * - If expected salinity is > 7ppt no salinity output is given
     *
     * @note If both the pH and specific conductance output is 99.99, check the
     * transducer health number for instruction.
     *
     * The specific conductance value is stored in holding register 0x0006
     * (decimal 6).
     *
     * @return The specific conductance value as a float. Passes the sensor
     * returned 99.99 for a bad value and returns -9999 if the sensor could not
     * be read.
     */
    float getSpecificConductance(void);

    /**
     * @brief Gets the current transducer health code from the sensor.
     *
     * The health code is stored in the lower byte of holding register 0x0008
     * (decimal 8).
     *
     * @return The transducer health code; passes ANBHealthCode::UNKNOWN (0xFF)
     * if the sensor could not be read.
     *
     * @see ANBHealthCode
     */
    ANBHealthCode getHealthCode(void);

    /**
     * @brief Get the transducer health code as a human readable string
     * @param code The health code as an ANBHealthCode
     * @return The health code as a string; "UNKNOWN" if the sensor could not be
     * read.
     */
    String getHealthString(ANBHealthCode code);
    /**
     * @brief Print a human-friendly translation of the transducer health code
     * to the serial monitor
     * @param code The health code to print
     * @param out The output stream to use (default is Serial)
     */
    void printHealthCode(ANBHealthCode code, Stream& out = Serial);

    /**
     * @brief Gets the current raw (non-temperature compensated) conductivity in
     * millisiemens per centimeter (mS/cm) from the sensor.
     *
     * @note If the raw conductivity output is 99.99 but the pH output is
     * OK, the salinity is out of range.
     * - Try changing your salinity setting
     * - If expected salinity is > 7ppt no salinity output is given
     *
     * @note If both the pH and raw conductivity output is 99.99, check the
     * transducer health number for instruction.
     *
     * The raw conductivity value is stored in holding register 0x0043 (decimal
     * 67).
     *
     * @return The raw (actual) conductivity value as a float. Passes the sensor
     * returned 99.99 for a bad value and returns -9999 if the sensor could not
     * be read.
     */
    float getRawConductivity(void);

    /**
     * @brief Gets the current status code from the sensor.
     *
     * @return The status code; passes ANBStatusCode::UNKNOWN (0xFF) if the
     * sensor could not be read.
     *
     * The status code is stored as the first digit of the two digit value
     * stored in the lower byte of holding register 0x0009 (decimal 9).
     *
     * @see ANBStatusCode
     */
    ANBStatusCode getStatusCode(void);
    /**
     * @brief Get the transducer status code as a human readable string
     * @param code The status code as an ANBStatusCode
     * @return The status code as a string; "UNKNOWN" if the sensor could not be
     * read.
     */
    String getStatusString(ANBStatusCode code);
    /**
     * @brief Print a human-friendly translation of the transducer status code
     * to the serial monitor
     * @param code The status code to print
     * @param out The output stream to use (default is Serial)
     */
    void printStatusCode(ANBStatusCode code, Stream& out = Serial);

    /**
     * @brief Gets the current diagnostics code from the sensor.
     *
     * @return The diagnostics code; passes ANBDiagnosticCode::UNKNOWN (0xFF) if
     * the sensor could not be read.
     *
     * The diagnostic code is stored as the second digit of the two digit value
     * stored in the lower byte of holding register 0x0009 (decimal 9).
     *
     * @see ANBDiagnosticCode
     */
    ANBDiagnosticCode getDiagnosticCode(void);
    /**
     * @brief Get the transducer diagnostic code as a human readable string
     * @param code The diagnostic code as an ANBDiagnosticCode
     * @return The diagnostic code as a string; "UNKNOWN" if the sensor could
     * not be read.
     */
    String getDiagnosticString(ANBDiagnosticCode code);
    /**
     * @brief Print a human-friendly translation of the transducer diagnostic
     * code to the serial monitor
     * @param code The diagnostic code to print
     * @param out The output stream to use (default is Serial)
     */
    void printDiagnosticCode(ANBDiagnosticCode code, Stream& out = Serial);

    /**
     * @brief Gets bulk values from all parameters
     *
     * All parameters *except the status code* can be read from 11 (0x0B)
     * holding registers starting at 0x0000.
     *
     * If the values could not be read, the function will return false and the
     * float output parameters (pH, temperature, salinity, specificConductance,
     * rawConductivity) will be set to -9999. The non-float output parameters
     * (status, diagnostics) will be set to 0xFF.
     *
     * @param pH The pH value
     * @param temperature The temperature value
     * @param salinity The salinity value
     * @param specificConductance The specific conductance value
     * @param rawConductivity The "actual" (raw) conductivity value
     * @param health The transducer health code
     * @param diagnostic The diagnostics code
     * @return True if the measurements were successfully obtained, false
     * if not.
     */
    bool getValues(float& pH, float& temperature, float& salinity,
                   float& specificConductance, float& rawConductivity,
                   ANBHealthCode& health, ANBDiagnosticCode& diagnostic);
    /**@}*/


    /**
     * @anchor admin_fxns
     * @name Admin Functions
     *
     * [Administrative
     * functions](https://www.anbsensors.com/newdocs/docs/modbus#admin-functions)
     * to get and set the sensor address, communication settings, and clock and
     * to get sensor metadata
     *
     * @note The Modbus instructions from ANB sensors state that the "source"
     * for several of the administrative functions are input (read only)
     * registers, but in all cases the functions use **holding** (read/write)
     * registers.  Most of the mislabeled registers are, however, **write only**
     * from within the modbus interface - attempting to read the registers will
     * give an illegal function error.
     */
    /**@{*/

    /**
     * @brief Enable Modbus communication on the sensor.
     *
     * The modbus enable command is in ~~input~~ **holding** register 0x0140
     * (decimal 320).
     *
     * Modbus is enabled immediately after the response
     *
     * @note The reverse of this command is enableTerminal()
     *
     * @warning This command does not seem to work properly!
     *
     * @return True if the operation was successful, false otherwise.
     */
    bool enableModbus();

    /**
     * @brief Attempt to force the sensor to enter Modbus mode by sending the
     * command via serial mode.
     */
    void forceModbus();

    /**
     * @brief Change to terminal (RS232) communication mode on the sensor.
     *
     * The terminal enable command is in ~~input~~ **holding** register 0x003B
     * (decimal 59).
     *
     * Terminal communication is enabled **on the next boot**.  If you need to
     * immediately switch to terminal mode without rebooting, you can send the
     * characters #700. This action prohibits the use of 0x23 (0x23 = #) as a
     * Modbus address.
     *
     * @note The reverse of this command is enableModbus()
     *
     * @return True if the operation was successful, false otherwise.
     */
    bool enableTerminal();

    /**
     * @brief Attempt to force the sensor to immediately enter terminal mode by
     * sending the #700 command.
     */
    void forceTerminal();

    /**
     * @brief Get the sensor modbus baud rate
     *
     * The baud rate is in the lower byte of ~~input~~ **holding** register
     * 0x003A (decimal 58).
     *
     * The factory default value is ANBSensorBaud::BAUD57600 (7), corresponding
     * to a baud rate of 57600.
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The baud rate is **write
     * only** from within the modbus interface.
     *
     * @return A code for the baud rate
     */
    ANBSensorBaud getBaud(void)
        __attribute__((error("Command not available!")));
    /**
     * @brief Set the sensor modbus baud
     *
     * The baud rate is in ~~input~~ **holding** register 0x003A (decimal 58).
     *
     * The new baud rate **does not take effect** until the next bootup.
     *
     * Change this value to any of the following valid values:
     *
     * ANBSensorBaud::BAUD9600 (1), ANBSensorBaud::BAUD14400 (2),
     * ANBSensorBaud::BAUD19200 (3), ANBSensorBaud::BAUD28800 (4),
     * ANBSensorBaud::BAUD38400 (5), ANBSensorBaud::BAUD56000 (6),
     * ANBSensorBaud::BAUD57600 (7), ANBSensorBaud::BAUD115200 (8)
     *
     * @param newSensorBaud The new baud rate to use
     * @return True if the baud rate was successfully set, false if not.
     */
    bool setBaud(ANBSensorBaud newSensorBaud);

    /**
     * @brief Gets the modbus sensor (slave) address.
     *
     * The address is in the lower byte of ~~input~~ **holding** register 0x0039
     * (decimal 57).
     *
     * @warning NOT YET SUPPORTED BY MODBUS COMMANDS. The modbus address is
     * **write only** from within the modbus interface.
     *
     * @return The modbus address of the ANB pH sensor
     */
    byte getAddress(void) __attribute__((error("Command not available!")));
    /**
     * @brief Set a new modbus sensor (slave) address.
     *
     * The address change will take effect on the next boot.
     *
     * @note The default Modbus address is 0x55 (decimal 85), but it can be any
     * number between 1 and 247 **except** 0x23 (decimal 35).  The address 0x23
     * is unusable because 0x23 is equivalent to the '#' character which is used
     * as the start of the return-to-terminal command.
     *
     * @warning The address 0x23 cannot be used!
     *
     * The address is in the lower byte of ~~input~~ **holding** register 0x0039
     * (decimal 57).
     *
     * @param newSensorAddress  The new address (slave ID) for the ANB pH sensor
     * @return True if the slave ID was successfully set, false if not.
     */
    bool setAddress(byte newSensorAddress);

    /**
     * @brief Gets the instrument serial number as a String
     *
     * The serial number takes up 3 holding registers starting at 0x000A
     * (decimal 10).
     *
     * @return The serial number of the ANB pH sensor
     */
    String getSerialNumber(void);

    /**
     * @brief Gets the instrument manufacturer as a String
     *
     * The manufacturer information takes up 8 holding registers starting at
     * 0x000D (decimal 13).
     *
     * @return The manufacturer of the ANB pH sensor; this should return "ANB
     * Sensors"
     */
    String getManufacturer(void);

    /**
     * @brief Gets the sensor name as a String
     *
     * The sensor name takes up 8 holding registers starting at 0x0015 (decimal
     * 21).
     *
     * @return The name of the ANB pH sensor
     */
    String getName(void);

    /**
     * @brief Gets the sensor sub-name as a String
     *
     * The sensor sub-name takes up 8 holding registers starting at 0x001D
     * (decimal 29).
     *
     * @return The sub-name of the ANB pH sensor
     */
    String getSubName(void);

    /**
     * @brief Gets the interface firmware (IF) version as a String
     *
     * [New firmware and firmware change logs are available
     * here.](https://www.anbsensors.com/newdocs/docs/Firmware/new-firmware)
     * [Older firmware is archived
     * here](https://www.anbsensors.com/newdocs/docs/Firmware/firmware-archive)
     *
     * The interface firmware (IF) version takes up 8 holding registers
     * starting at 0x0025 (decimal 37).
     *
     * @return The interface firmware (IF)version of the ANB pH sensor
     */
    String getInterfaceVersion(void);

    /**
     * @brief Gets the driver (DV) firmware version as a String
     *
     * [New firmware and firmware change logs are available
     * here.](https://www.anbsensors.com/newdocs/docs/Firmware/new-firmware)
     * [Older firmware is archived
     * here](https://www.anbsensors.com/newdocs/docs/Firmware/firmware-archive)
     *
     * The driver (DV) firmware version takes up 8 holding registers starting at
     * 0x002D (decimal 45).
     *
     * @return The driver (DV) firmware version of the ANB pH sensor
     */
    String getDriverVersion(void);

    /**
     * @brief Gets the current RTC (Real-Time Clock) value on the sensor
     *
     * The RTC value is stored in BCD (Binary Coded Decimal)in 6 holding
     * registers starting at 0x003D (decimal 61).
     *
     * @return True if the RTC value was successfully retrieved, false
     * otherwise.
     *
     * @param seconds Reference to a variable where the seconds will be stored
     * @param minutes Reference to a variable where the minutes will be stored
     * @param hours Reference to a variable where the hours will be stored
     * @param day Reference to a variable where the day will be stored
     * @param month Reference to a variable where the month will be stored
     * @param year Reference to a variable where the year will be stored
     */
    bool getRTC(int8_t& seconds, int8_t& minutes, int8_t& hours, int8_t& day,
                int8_t& month, int16_t& year);
    /**
     * @brief Set a new RTC (Real-Time Clock) value on the sensor.
     *
     * The RTC value is stored in BCD (Binary Coded Decimal) in 6 holding
     * registers starting at 0x003D (decimal 61).
     *
     * @param seconds The seconds portion of the current time
     * @param minutes The minutes portion of the current time
     * @param hours The hours portion of the current time
     * @param day The day portion of the current time
     * @param month The month portion of the current time
     * @param year The year portion of the current time
     * @return True if the RTC value was successfully set, false otherwise.
     */
    bool setRTC(int8_t seconds, int8_t minutes, int8_t hours, int8_t day,
                int8_t month, int16_t year);
    /**@}*/

    /**
     * @anchor debugging
     * @name Debugging functions
     *
     * @note These functions control the debugging output of the Modbus
     * communication. They are intended for debugging the communication and
     * Modbus commands. These functions should not be used in production code as
     * they can affect performance.
     *
     * @warning These functions are **not** for debugging the actual operation
     * and quality of the sensor values!
     */
    /**@{*/

    /**
     * @brief Set a stream for debugging information to go to.
     *
     * @param stream An Arduino stream object
     */
    void setDebugStream(Stream* stream) {
        modbus.setDebugStream(stream);
        _debugStream = stream;
    }
    /**
     * @copydoc anbSensor::setDebugStream(Stream* stream)
     */
    void setDebugStream(Stream& stream) {
        modbus.setDebugStream(stream);
        _debugStream = &stream;
    }
    /**
     * @brief Un-set the stream for debugging information to go to; stop
     * debugging.
     */
    void stopDebugging(void) {
        modbus.stopDebugging();
        _debugStream = nullptr;
    }
    /**@}*/

 private:
    byte _slaveID;  ///< the sensor slave id
    /**
     * @brief The stream instance (serial port) for communication with the
     * Modbus slave (usually over RS485)
     *
     * We keep a pointer to this stream from the input so that we can use it for
     * forceTerminal() and forceModbus() functions. The stream pointer from the
     * SensorModbusMaster library is private.
     */
    Stream* _stream;
    /**
     * @brief Internal reference to the Modbus communication object from
     * EnviroDIY SensorModbusMaster library.
     *
     * @see https://github.com/EnviroDIY/SensorModbusMaster
     */
    modbusMaster
        modbus;  ///< an internal reference to the modbus communication object.

    void setDefaultTimeouts();

    /**
     * @brief The stream instance (serial port) for debugging
     */
    Stream* _debugStream;

    // Utility templates for writing to the debugging stream
    template <typename T>
    inline void debugPrint(T last) {
        if (_debugStream != nullptr) {
            _debugStream->print(last);
            _debugStream->flush();
        }
    }
    template <typename T, typename... Args>
    inline void debugPrint(T head, Args... tail) {
        if (_debugStream != nullptr) {
            _debugStream->print(head);
            this->debugPrint(tail...);
        }
    }
    void dumpToDebugStream(uint32_t wait = 1000L) {
        uint32_t startTime = millis();
        while (millis() - startTime < wait && _stream->available() < 10);
        do {
            if (_debugStream != nullptr) {
                _debugStream->println(_stream->readStringUntil('\n'));
            } else {
                _stream->readStringUntil('\n');
            }
        } while (_stream->available());
    }
};

#endif
