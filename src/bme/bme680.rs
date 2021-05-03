//! Provides a [`BmeSensor`] implementation for the BME680 sensor.
//!
//! This module is only available if the **use-bme680** feature is enabled.
//!
//! The implementation is based on the
//! [bme680 crate](https://crates.io/crates/bme680).

use std::fmt::Debug;
use std::time::{Duration, Instant};

use crate::bme::{BmeSensor, BmeSettingsHandle};
use crate::{Input, InputKind};
use bme680::{Bme680, OversamplingSetting, PowerMode, SettingsBuilder};
use embedded_hal::blocking::{delay::DelayMs, i2c};

/// Implementation of the [`BmeSensor`] trait for the BME680 sensor.
///
/// For the gas resistance measurement the last reading of the temperature
/// sensor will be used as ambient temperature.
pub struct Bme680Sensor<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    bme680: Bme680<I2C, D>,
    measurement_available_after: Option<Instant>,
    last_measured_temperature_celsius: f32,
}

impl<I2C, D> Bme680Sensor<I2C, D>
where
    I2C: i2c::Read + i2c::Write,
    D: DelayMs<u8>,
{
    /// Create a new instance using *bme680*.
    ///
    /// *initial_ambient_temperature_celsius* sets the ambient temperature for
    /// the first gas resistance reading. All subsequent readings use the
    /// respective last temperature reading.
    pub fn new(bme680: Bme680<I2C, D>, initial_ambient_temperature_celsius: f32) -> Self {
        Bme680Sensor {
            bme680,
            measurement_available_after: None,
            last_measured_temperature_celsius: initial_ambient_temperature_celsius,
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
                self.last_measured_temperature_celsius.round() as i8,
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
                self.last_measured_temperature_celsius = data.temperature_celsius();
                Ok(vec![
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
                ])
            }
        }
    }
}
