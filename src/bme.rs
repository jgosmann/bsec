//! TODO

use super::Input;
use libalgobsec_sys::bsec_bme_settings_t;
use std::{fmt::Debug, time::Duration};

/// Handle to a struct with settings for the BME sensor.
///
/// TODO: add example how to configure BME sensor
pub struct BmeSettingsHandle<'a> {
    bme_settings: &'a bsec_bme_settings_t,
}

impl<'a> BmeSettingsHandle<'a> {
    pub(crate) fn new(bme_settings: &'a bsec_bme_settings_t) -> Self {
        Self { bme_settings }
    }

    pub fn heater_temperature(&self) -> u16 {
        self.bme_settings.heater_temperature
    }

    pub fn heating_duration(&self) -> u16 {
        self.bme_settings.heating_duration
    }

    pub fn run_gas(&self) -> bool {
        self.bme_settings.run_gas == 1
    }

    pub fn pressure_oversampling(&self) -> u8 {
        self.bme_settings.pressure_oversampling
    }

    pub fn temperature_oversampling(&self) -> u8 {
        self.bme_settings.temperature_oversampling
    }

    pub fn humidity_oversampling(&self) -> u8 {
        self.bme_settings.humidity_oversampling
    }
}

/// Trait to implement for your specific hardware to obtain measurements from
/// the BME sensor.
///
/// TODO add example
pub trait BmeSensor {
    /// Error type if an operation with the sensor fails.
    type Error: Debug;

    /// Start a sensor measurement.
    ///
    /// * `settings`: Settings specifying the measurement protocol.
    ///
    /// Shoud returns the duration after which the measurement will be available
    /// or an error.
    fn start_measurement(&mut self, settings: &BmeSettingsHandle) -> Result<Duration, Self::Error>;

    /// Read a finished sensor measurement.
    ///
    /// Returns the sensor measurements as a vector with an item for each
    /// physical sensor read.
    ///
    /// To compensate for heat sources near the sensor add an additional output
    /// to the vector, using the sensor type [`BsecInputKind::HeatSource`]
    /// and the desired correction in degrees Celsius.
    fn get_measurement(&mut self) -> nb::Result<Vec<Input>, Self::Error>;
}

#[cfg(any(test, feature = "test_support"))]
pub mod test_support {
    use super::*;

    #[derive(Copy, Clone, Debug)]
    pub struct UnitError;

    impl std::error::Error for UnitError {}

    impl std::fmt::Display for UnitError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
            f.write_fmt(format_args!("{:?}", self))
        }
    }

    pub struct FakeBmeSensor {
        measurement: nb::Result<Vec<Input>, UnitError>,
    }

    impl FakeBmeSensor {
        pub fn new(measurement: nb::Result<Vec<Input>, UnitError>) -> Self {
            Self { measurement }
        }
    }

    impl Default for FakeBmeSensor {
        fn default() -> Self {
            Self::new(Ok(vec![]))
        }
    }

    impl BmeSensor for FakeBmeSensor {
        type Error = UnitError;
        fn start_measurement(
            &mut self,
            _: &BmeSettingsHandle<'_>,
        ) -> Result<std::time::Duration, UnitError> {
            Ok(std::time::Duration::new(0, 0))
        }
        fn get_measurement(&mut self) -> nb::Result<Vec<Input>, UnitError> {
            self.measurement.clone()
        }
    }
}

#[cfg(feature = "use-bme680")]
pub mod bme680;
