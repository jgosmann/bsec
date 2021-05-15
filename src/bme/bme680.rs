//! Provides a [`BmeSensor`] implementation for the BME680 sensor.
//!
//! This module is only available if the **use-bme680** feature is enabled.
//!
//! The implementation is based on the
//! [bme680 crate](https://crates.io/crates/bme680).
//!
//! # Example
//!
//! ```ignore
//! use bme680::{Bme680, I2CAddress};
//! use bsec::bme::bme680::Bme680Sensor;
//! use linux_embedded_hal::{Delay, I2cdev};
//!
//! let i2c = I2cdev::new("/dev/i2c")?;
//! let bme680 = Bme680::init(i2c, Delay {}, I2CAddress::Primary)?;
//! let sensor = Bme680SensorBuilder::new(bme680)
//!     .with_initial_ambient_temp_celsius(25.)
//!     .with_temp_offset_celsius(6.3)
//!     .build();
//! ```

use std::fmt::Debug;
use std::time::{Duration, Instant};

use crate::bme::{BmeSensor, BmeSettingsHandle};
use crate::{Input, InputKind};
use bme680::{Bme680, OversamplingSetting, PowerMode, SettingsBuilder};
use embedded_hal::blocking::{delay::DelayMs, i2c};

/// Builder for [`Bme680Sensor`] instances.
pub struct Bme680SensorBuilder<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    bme680: Bme680<I2C, D>,
    initial_ambient_temp_celsius: f32,
    temp_offset_celsius: f32,
    disable_baseline_tracker: Option<f32>,
}

impl<I2C, D> Bme680SensorBuilder<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    /// Create a new builder instance with `bme680`.
    ///
    /// Uses the following defaults:
    ///
    /// * `initial_ambient_temp_celsius = 20.0`
    /// * `temp_offset_celsius = 0.0`
    /// * `disable_baseline_tracker = false`
    pub fn new(bme680: Bme680<I2C, D>) -> Self {
        Self {
            bme680,
            initial_ambient_temp_celsius: 20.,
            temp_offset_celsius: 0.,
            disable_baseline_tracker: None,
        }
    }

    /// Sets the initial ambient temperature in Celsius to `value`.
    pub fn initial_ambient_temp_celsius(mut self, value: f32) -> Self {
        self.initial_ambient_temp_celsius = value;
        self
    }

    /// Sets the temperature offset in Celsius to `value`.
    pub fn temp_offset_celsius(mut self, value: f32) -> Self {
        self.temp_offset_celsius = value;
        self
    }

    /// Set the status of the BSEC baseline tracker to `status`.
    pub fn disable_baseline_tracker(mut self, status: Option<f32>) -> Self {
        self.disable_baseline_tracker = status;
        self
    }

    /// Consume current configuration and create [`Bme680Sensor`] instance.
    pub fn build(self) -> Bme680Sensor<I2C, D> {
        Bme680Sensor::new(
            self.bme680,
            self.initial_ambient_temp_celsius,
            self.temp_offset_celsius,
            self.disable_baseline_tracker,
        )
    }
}

/// Implementation of the [`BmeSensor`] trait for the BME680 sensor.
///
/// For the gas resistance measurement, the last reading of the temperature
/// sensor will be used as ambient temperature.
///
/// Use [`Bme680SensorBuilder`] to create instances of this struct.
pub struct Bme680Sensor<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    bme680: Bme680<I2C, D>,
    measurement_available_after: Option<Instant>,
    last_measured_temp_celsius: f32,
    temp_offset_celsius: f32,
    disable_baseline_tracker: Option<f32>,
}

impl<I2C, D> Bme680Sensor<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    /// Create a new instance reading measurement from `bme680`.
    ///
    /// * `initial_ambient_temp_celsius` sets the ambient temperature for
    ///   the first gas resistance reading. All subsequent readings use the
    ///   respective last temperature reading.
    /// * `temp_offset_celsius` sets the temperature offset for the sensor to
    ///   its surroundings due to heat sources in proximity. The value will be
    ///   provided to the BSEC algorithm as [`crate::InputKind::HeatSource`].
    /// * 'disable_baseline_tracker` controls the BSEC baseline tracker if set.
    ///   The value will be provided to the BSEC algorithm as
    ///   [`crate::InputKind::DisableBaselineTracker`]. The the *Bosch BSEC*
    ///   documentation for details.
    fn new(
        bme680: Bme680<I2C, D>,
        initial_ambient_temp_celsius: f32,
        temp_offset_celsius: f32,
        disable_baseline_tracker: Option<f32>,
    ) -> Self {
        Bme680Sensor {
            bme680,
            measurement_available_after: None,
            last_measured_temp_celsius: initial_ambient_temp_celsius,
            temp_offset_celsius,
            disable_baseline_tracker,
        }
    }
}

impl<I2C, D> BmeSensor for Bme680Sensor<I2C, D>
where
    D: DelayMs<u8>,
    I2C: i2c::Read + i2c::Write,
    <I2C as i2c::Read>::Error: Debug,
    <I2C as i2c::Write>::Error: Debug,
{
    type Error = bme680::Error<<I2C as i2c::Read>::Error, <I2C as i2c::Write>::Error>;

    fn start_measurement(&mut self, settings: &BmeSettingsHandle) -> Result<Duration, Self::Error> {
        let settings = SettingsBuilder::new()
            .with_humidity_oversampling(OversamplingSetting::from_u8(
                settings.humidity_oversampling(),
            ))
            .with_temperature_oversampling(OversamplingSetting::from_u8(
                settings.temperature_oversampling(),
            ))
            .with_pressure_oversampling(OversamplingSetting::from_u8(
                settings.pressure_oversampling(),
            ))
            .with_run_gas(settings.run_gas())
            .with_gas_measurement(
                Duration::from_millis(settings.heating_duration().into()),
                settings.heater_temperature(),
                self.last_measured_temp_celsius.round() as i8,
            )
            .build();

        self.bme680.set_sensor_settings(settings)?;
        let profile_duration = self.bme680.get_profile_dur(&settings.0)?;
        self.bme680.set_sensor_mode(PowerMode::ForcedMode)?;
        self.measurement_available_after = Some(Instant::now() + profile_duration);
        Ok(profile_duration)
    }

    fn get_measurement(&mut self) -> nb::Result<Vec<Input>, Self::Error> {
        match self.measurement_available_after {
            None => panic!("must call start_measurement before get_measurement"),
            Some(instant) if instant > Instant::now() => Err(nb::Error::WouldBlock),
            _ => {
                let (data, _state) = self.bme680.get_sensor_data()?;
                self.last_measured_temp_celsius = data.temperature_celsius();
                let mut bsec_inputs = Vec::with_capacity(6);
                bsec_inputs.extend(&[
                    Input {
                        sensor: InputKind::Temperature,
                        signal: data.temperature_celsius(),
                    },
                    Input {
                        sensor: InputKind::Pressure,
                        signal: data.pressure_hpa(),
                    },
                    Input {
                        sensor: InputKind::Humidity,
                        signal: data.humidity_percent(),
                    },
                    Input {
                        sensor: InputKind::GasResistor,
                        signal: data.gas_resistance_ohm() as f32,
                    },
                    Input {
                        sensor: InputKind::HeatSource,
                        signal: self.temp_offset_celsius,
                    },
                ]);
                if let Some(status) = self.disable_baseline_tracker {
                    bsec_inputs.push(Input {
                        sensor: InputKind::DisableBaselineTracker,
                        signal: status,
                    });
                }
                Ok(bsec_inputs)
            }
        }
    }
}
