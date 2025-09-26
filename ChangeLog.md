# ChangeLog<!--! {#change_log} -->

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) and its stricter, better defined, brother [Common Changelog](https://common-changelog.org/).

This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

***

## [Unreleased]

### Changed

### Added

### Removed

### Fixed

***

## [0.2.6]

### Fixed

- Always check that a stream is not null before using it.

### Changed

- Bumped dependencies

### Removed

- Removed non-functional findBaud function.

***

## [0.2.5]

### Added

- Added a function to attempt to automatically find the baud rate of a sensor, though it's only partially functional.

### Changed

- Require a stream as input to print out human-readable output.


## [0.2.4]

### Changed

- Bumped SensorModbusMaster dependency to 1.6.5

***

## [0.2.3]

### Added

- Added Rob Tillaart's fast_math library as a dependency for BCD conversions

### Fixed

- Fixed the RTC set functions to use BCD

***

## [0.2.2]

### Changed

- Bumped SensorModbusMaster dependency

***

## [0.2.1]

### Added

- Add command for writing bulk configuration

***

## [0.2.0]

### Changed

- **BREAKING** The bulk getValues function no longer has the status code as an argument
  - The bulk call does not return the status code, so I removed it.

### Fixed

- Corrected order of bytes in the getValues function.

***

## [0.1.1]

### Changed

- Bumped SensorModbusMaster dependency to 1.4.0

### Added

- Added overload constructors

***

## [0.1.0]

Initial release

***

[Unreleased]: https://github.com/EnviroDIY/ANBSensorsModbus/compare/v0.2.5...HEAD
[0.2.5]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.5
[0.2.4]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.4
[0.2.3]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.3
[0.2.2]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.2
[0.2.1]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.1
[0.2.0]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.2.0
[0.1.1]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.1.1
[0.1.0]: https://github.com/EnviroDIY/ANBSensorsModbus/releases/tag/v0.1.0

<!--! @tableofcontents{HTML:1} -->

<!--! @m_footernavigation -->
