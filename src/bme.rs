//! Interface to BME sensors to read physical measurements.

use super::Input;
#[cfg(not(feature = "docs-rs"))]
use libalgobsec_sys::bsec_bme_settings_t;
use std::{fmt::Debug, time::Duration};

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_bme_settings_t {}

/// Trait to implement for your specific hardware to obtain measurements from
/// the BME sensor.
///
/// # Example
///
/// An rudimentary implementation for the BME680 sensor with the
/// [bme680](https://crates.io/crates/bme680) crate might look like this:
///
/// ```
/// use bme680::{Bme680, OversamplingSetting, PowerMode, SettingsBuilder};
/// use bsec::{Input, InputKind};
/// use bsec::bme::{BmeSensor, BmeSettingsHandle};
/// use embedded_hal::blocking::{delay::DelayMs, i2c};
/// use std::fmt::Debug;
/// use std::time::Duration;
///
/// pub struct Bme680Sensor<I2C, D>
/// where
///     D: DelayMs<u8>,
///     I2C: i2c::Read + i2c::Write
/// {
///     bme680: Bme680<I2C, D>,
///     delay: D,
/// }
///
/// impl<I2C, D> BmeSensor for Bme680Sensor<I2C, D>
/// where
///     D: DelayMs<u8>,
///     I2C: i2c::Read + i2c::Write,
///     <I2C as i2c::Read>::Error: Debug,
///     <I2C as i2c::Write>::Error: Debug,
/// {
///     type Error = bme680::Error<<I2C as i2c::Read>::Error, <I2C as i2c::Write>::Error>;
///     
///     fn start_measurement(
///         &mut self,
///         settings: &BmeSettingsHandle,
///     ) -> Result<std::time::Duration, Self::Error> {
///         let settings = SettingsBuilder::new()
///             .with_humidity_oversampling(OversamplingSetting::from_u8(
///                 settings.humidity_oversampling(),
///             ))
///             .with_temperature_oversampling(OversamplingSetting::from_u8(
///                 settings.temperature_oversampling(),
///             ))
///             .with_pressure_oversampling(OversamplingSetting::from_u8(
///                 settings.pressure_oversampling(),
///             ))
///             .with_run_gas(settings.run_gas())
///             .with_gas_measurement(
///                 Duration::from_millis(settings.heating_duration().into()),
///                 settings.heater_temperature(),
///                 20,
///             )
///             .build();
///         self.bme680
///             .set_sensor_settings(&mut self.delay, settings)?;
///         let profile_duration = self.bme680.get_profile_dur(&settings.0)?;
///         self.bme680.set_sensor_mode(&mut self.delay, PowerMode::ForcedMode)?;
///         Ok(profile_duration)
///     }
///
///     fn get_measurement(&mut self) -> nb::Result<Vec<Input>, Self::Error> {
///         let (data, _state) = self.bme680.get_sensor_data(&mut self.delay)?;
///         Ok(vec![
///             Input {
///                 sensor: InputKind::Temperature,
///                 signal: data.temperature_celsius(),
///             },
///             Input {
///                 sensor: InputKind::Pressure,
///                 signal: data.pressure_hpa(),
///             },
///             Input {
///                 sensor: InputKind::Humidity,
///                 signal: data.humidity_percent(),
///             },
///             Input {
///                 sensor: InputKind::GasResistor,
///                 signal: data.gas_resistance_ohm() as f32,
///             },
///         ])
///     }
/// }
/// ```
pub trait BmeSensor {
    /// Error type if an operation with the sensor fails.
    type Error: Debug;

    /// Starts a sensor measurement.
    ///
    /// Use `settings` to configure your BME sensor as requested by the BSEC
    /// algorithm.
    ///
    /// Shoud return the duration after which the measurement will be available
    /// or an error.
    fn start_measurement(&mut self, settings: &BmeSettingsHandle) -> Result<Duration, Self::Error>;

    /// Read a finished sensor measurement.
    ///
    /// Returns the sensor measurements as a vector with an item for each
    /// physical sensor read.
    ///
    /// To compensate for heat sources near the sensor, add an additional output
    /// to the vector, using the sensor type [`super::InputKind::HeatSource`]
    /// and the desired correction in degrees Celsius.
    fn get_measurement(&mut self) -> nb::Result<Vec<Input>, Self::Error>;
}

/// Handle to a struct with settings for the BME sensor.
///
/// Retrieve the settings from this handle to configure your BME sensor
/// appropriately in [`BmeSensor::start_measurement`] for the measurements
/// requested by the BSEC algorithm.
pub struct BmeSettingsHandle<'a> {
    bme_settings: &'a bsec_bme_settings_t,
}

impl<'a> BmeSettingsHandle<'a> {
    pub(crate) fn new(bme_settings: &'a bsec_bme_settings_t) -> Self {
        Self { bme_settings }
    }

    /// Returns the desired gas sensor heater target temperature.
    pub fn heater_temperature(&self) -> u16 {
        self.bme_settings.heater_temperature
    }

    /// Returns the desired gas sensor heating duration in milliseconds.
    pub fn heating_duration(&self) -> u16 {
        self.bme_settings.heating_duration
    }

    /// Returns whether to run a gas measurement.
    pub fn run_gas(&self) -> bool {
        self.bme_settings.run_gas == 1
    }

    /// Returns the desired oversampling of barometric pressure measurements.
    pub fn pressure_oversampling(&self) -> u8 {
        self.bme_settings.pressure_oversampling
    }

    /// Returns the desired oversampling of temperature measurements.
    pub fn temperature_oversampling(&self) -> u8 {
        self.bme_settings.temperature_oversampling
    }

    /// Returns the desired oversampling of humidity measurements.
    pub fn humidity_oversampling(&self) -> u8 {
        self.bme_settings.humidity_oversampling
    }
}

#[cfg(any(test, feature = "test-support"))]
pub mod test_support;

#[cfg(feature = "use-bme680")]
pub mod bme680;
