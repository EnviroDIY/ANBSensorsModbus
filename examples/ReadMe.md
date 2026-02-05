# Examples using the ANB Sensors Modbus Library<!--! {#page_the_examples} -->

These example programs demonstrate how to use the ANB Sensors Modbus library.

___

<!--! @if GITHUB -->

- [Examples using the ANB Sensors Modbus Library](#examples-using-the-anb-sensors-modbus-library)
  - [Getting Sensor Values](#getting-sensor-values)
  - [Abrading the Sensor](#abrading-the-sensor)
  - [Using the sensor in Autonomous Mode](#using-the-sensor-in-autonomous-mode)
  - [Downloading Internal SD Card Data](#downloading-internal-sd-card-data)
  - [Finding the current sensor baud rate](#finding-the-current-sensor-baud-rate)

<!--! @endif -->

<!--! @tableofcontents -->

<!--! @m_footernavigation -->

## Getting Sensor Values<!--! {#examples_get_values} -->

This prints basic meta-data about a sensor to the first serial port and then begins taking measurements from the sensor.

- [Instructions for the get values example](https://envirodiy.github.io/ANBSensorsModbus/example_get_values.html)
- [The get values example on GitHub](https://github.com/EnviroDIY/ANBSensorsModbus/tree/master/examples/GetValues)

<!--! @m_innerpage{example_get_values} -->

## Abrading the Sensor<!--! {#examples_abrade_sensor} -->

This informs the sensor that it has been abraded and that it should clear its internal references.
This DOES NOT actually abrade the sensor!
The actual abrasion is done with a sanding disc per the manufactures instructions

- [Instructions for the abrasion example](https://envirodiy.github.io/ANBSensorsModbus/example_abrasion.html)
- [The abrasion example on GitHub](https://github.com/EnviroDIY/ANBSensorsModbus/tree/master/examples/AbradeSensor)

<!--! @m_innerpage{example_abrasion} -->

## Using the sensor in Autonomous Mode<!--! {#examples_test_autonomous} -->

This puts the sensor into autonomous mode and relays live printouts of scans.

- [Instructions for the download files example](https://envirodiy.github.io/ANBSensorsModbus/example_test_autonomous.html)
- [The download files example on GitHub](https://github.com/EnviroDIY/ANBSensorsModbus/tree/master/examples/TestAutonomous)

<!--! @m_innerpage{example_test_autonomous} -->

## Downloading Internal SD Card Data<!--! {#examples_download_data} -->

This uses the terminal prompt to transfer data from the serial card built into the pH sensor to an SD card on the Arduino.
This does not delete the files from the internal SD card.

- [Instructions for the download files example](https://envirodiy.github.io/ANBSensorsModbus/example_download_data.html)
- [The download files example on GitHub](https://github.com/EnviroDIY/ANBSensorsModbus/tree/master/examples/DownloadData)

<!--! @m_innerpage{example_download_data} -->

## Finding the current sensor baud rate<!--! {#examples_find_baud} -->

This tries all supported baud rates to attempt to communicate with the sensor.

- [Instructions for the download files example](https://envirodiy.github.io/ANBSensorsModbus/example_find_baud.html)
- [The download files example on GitHub](https://github.com/EnviroDIY/ANBSensorsModbus/tree/master/examples/FindSensorBaud)

<!--! @m_innerpage{example_find_baud} -->
