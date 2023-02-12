# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.0] - 2023-02-12

### Changed

* Update dependencies. In particular, [bme680](https://crates.io/crates/bme680)
  to version 0.6.0 where methods now requires an additional `delay` argument.


## [0.3.1] - 2021-11-09

### Fixed

* Memory orderings for aquireing and releasing BSEC have been slightly relaxed.
* Memory orderings for the `FakeClock` (only used in tests) have been relaxed.


## [0.3.0] - 2021-05-15

### Added

* Support for the `HeatSource` and `DisableBaselineTracker` *Bosch BSEC* inputs
  in the `Bme680Sensor`.
* `SubscriptionRequest` is now hashable.

### Changed

* The `Bme680Sensor` cannot be instantiated directly anymore, but is created
  with the `Bme680SensorBuilder`.


## [0.2.0] - 2021-05-10

### Changed

* The `bsec::clock::test_support::FakeClock` is now `Sync`.


## [0.1.2] - 2021-05-04

Try to get documentation to build on docs.rs.


## [0.1.1] - 2021-05-04

Try to get documentation to build on docs.rs.


## [0.1.0] - 2021-05-04

Initial release.


[Unreleased]: https://github.com/jgosmann/bsec/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/jgosmann/bsec/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/jgosmann/bsec/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/jgosmann/bsec/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/jgosmann/bsec/compare/v0.1.2...v0.2.0
[0.1.2]: https://github.com/jgosmann/bsec/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/jgosmann/bsec/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/jgosmann/bsec/releases/tag/v0.1.0
