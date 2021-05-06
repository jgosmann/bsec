//! Rust API to the
//! [Bosch BSEC library](https://www.bosch-sensortec.com/software-tools/software/bsec/).
//!
//! This documentation will use *bsec* to refer to this crate, while
//! *Bosch BSEC* is used to refer to the original BSEC library provided by
//! Bosch.
//!
//! ## Important license information
//!
//! The *Bosch BSEC* library is proprietary. Thus, the *Bosch BSEC* library and
//! its documentation cannot be included in the *bsec* Rust crate and need to be
//! obtained separately.
//!
//! While the *bsec* documentation covers the Rust crate itself, you will likely
//! have to refer to the *Bosch BSEC* documentation at some points to get a full
//! understanding.
//!
//! You are responsible for adhering to the Bosch BSEC lincese terms in your
//! products, despite the Rust API in this crate being published under a
//! permissive license.
//!
//! * [Bosch BSEC website to obtain your copy](https://www.bosch-sensortec.com/software-tools/software/bsec/)
//! * [Bosch BESC license terms at the time of writing](https://www.bosch-sensortec.com/media/boschsensortec/downloads/bsec/2017-07-17_clickthrough_license_terms_environmentalib_sw_clean.pdf)
//!
//!
//! ## Getting started
//!
//! ### Setup paths to the Bosch BSEC library
//!
//! To be able to use this crate, it needs to know where to find the
//! *Bosch BSEC* header files and library on your system. These paths are
//! provided as the configuration options `besc_include_path` and
//! `bsec_library_path` to the Rust compiler.
//!
//! You can do this by creating a `.cargo/config` file in your crate with the
//! following content (adjust the paths accordingly):
//!
//! ```toml
//! [build]
//! rustflags = [
//!     '--cfg', 'bsec_include_path="/path/to/BSEC_1.4.8.0_Generic_Release/algo/normal_version/inc"',
//!     '--cfg', 'bsec_library_path="/path/to/BSEC_1.4.8.0_Generic_Release/algo/normal_version/bin/target-arch"',
//! ]
//! ```
//!
//! (You might want to also have a look at the instructions for
//! [libalgobsec-sys](https://crates.io/crates/libalgobsec-sys) providing the
//! actual low-level bindings.)
//!
//! ### Implement necessary traits
//!
//! To be able to use the *bsec* crate, you need implementations of the
//! [`Clock`] and [`BmeSensor`] traits
//!
//! #### Clock
//!
//! The [`Clock`] traits allows the BSEC algorithm to obtain timestamps to
//! schedule sensor measurements accordingly. Your implementation might depend
//! on your hardware platform or you can use the generic implementation
//! [`clock::TimePassed`].
//!
//! #### BmeSensor
//!
//! The [`BmeSensor`] trait allows the BSEC algorithm to communicate with your
//! BME sensor and obtain measurements. You can implement it yourself or use
//! a ready-made implementation shipped with *bsec*:
//!
//! * **BME680** implementation is provided as [`bme::bme680::Bme680Sensor`];
//!   requires the **use-bme680** feature.
//!
//!
//! ### Usage
//!
//! The following example demonstrates the basic *bsec* usage. Essentially, you
//! need to
//!
//! * acquire an instance of the library (only one can be in use at any given
//!   time),
//! * subscribe to the desired outputs,
//! * and perform measurements in accordance with the sampling rate.
//!
//! The outputs of the BSEC algorithm are also considered virtual sensors.
//! The inputs to the BSEC algorithm are the sensor measurements and are
//! considered physical sensors (though there are some special input that do
//! not actually correspond to any physical sensor).
//!
//! ```
//! use bsec::{Bsec, Input, InputKind, OutputKind, clock::Clock, SampleRate, SubscriptionRequest};
//! use nb::block;
//! use std::time::Duration;
//! #
//! # #[cfg(not(feature = "test-support"))]
//! # fn main() { panic!("doctests must be run with `test-support` feature.") }
//! #
//! # #[cfg(feature = "test-support")]
//! # type TimePassed = bsec::clock::test_support::FakeClock;
//! #
//! # fn sleep_for(duration: Duration) -> () {}
//! #
//! # #[cfg(feature = "test-support")]
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! #
//! # let clock = TimePassed::default();
//! # let sensor = bsec::bme::test_support::FakeBmeSensor::new(Ok(vec![
//! #    Input {
//! #         sensor: InputKind::Temperature,
//! #         signal: 22.,
//! #     },
//! #     Input {
//! #         sensor: InputKind::Humidity,
//! #         signal: 40.,
//! #     },
//! #     Input {
//! #         sensor: InputKind::Pressure,
//! #         signal: 1000.,
//! #     },
//! #     Input {
//! #         sensor: InputKind::GasResistor,
//! #         signal: 6000.,
//! #     },
//! # ]));
//!
//! // Acquire handle to the BSEC library.
//! // Only one such handle can be acquired at any time.
//! let mut bsec: Bsec<_, TimePassed, _> = Bsec::init(sensor, &clock)?;
//!
//! // Configure the outputs you want to subscribe to.
//! bsec.update_subscription(&[
//!     SubscriptionRequest {
//!         sample_rate: SampleRate::Lp,
//!         sensor: OutputKind::Iaq,
//!     },
//! ])?;
//!
//! // We need to feed BSEC regularly with new measurements.
//! loop {
//!     // Wait for when the next measurement is due.
//!     sleep_for(Duration::from_nanos((bsec.next_measurement() - clock.timestamp_ns()) as u64));
//!
//!     // Start the measurement.
//!     let wait_duration = block!(bsec.start_next_measurement())?;
//!     sleep_for(wait_duration);
//!     # clock.advance_by(wait_duration);
//!
//!     // Process the measurement when ready and print the BSEC outputs.
//!     let outputs = block!(bsec.process_last_measurement())?;
//!     for output in &outputs {
//!         println!("{:?}: {}", output.sensor, output.signal);
//!     }
//! #
//! #   let signals: std::collections::HashMap<bsec::OutputKind, &bsec::Output> =
//! #       outputs.iter().map(|s| (s.sensor, s)).collect();
//! #   assert!(
//! #       (signals.get(&bsec::OutputKind::Iaq).unwrap().signal - 25.).abs()
//! #           < f64::EPSILON
//! #   );
//! #   return Ok(())
//! }
//! # }
//! ```
//!
//! ## Features
//!
//! * **use-bme680**: Enables the [`bme680`] module providing a [`BmeSensor`]
//!   implementation for the BME680 sensor to use it with *bsec*.
//! * **test-support**: Provides additional classes for unit testing.

use crate::bme::{BmeSensor, BmeSettingsHandle};
use crate::clock::Clock;
use crate::error::{BsecError, ConversionError, Error};
#[cfg(not(feature = "docs-rs"))]
use libalgobsec_sys::{
    bsec_bme_settings_t, bsec_do_steps, bsec_get_configuration, bsec_get_state, bsec_get_version,
    bsec_init, bsec_input_t, bsec_library_return_t, bsec_output_t, bsec_physical_sensor_t,
    bsec_reset_output, bsec_sensor_configuration_t, bsec_sensor_control, bsec_set_configuration,
    bsec_set_state, bsec_update_subscription, bsec_version_t, bsec_virtual_sensor_t,
    BSEC_MAX_PHYSICAL_SENSOR, BSEC_MAX_PROPERTY_BLOB_SIZE, BSEC_MAX_STATE_BLOB_SIZE,
    BSEC_MAX_WORKBUFFER_SIZE,
};
use std::borrow::Borrow;
use std::convert::{From, TryFrom, TryInto};
use std::fmt::Debug;
use std::hash::Hash;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_library_return_t {}

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_output_t {}

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_physical_sensor_t {}

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_sensor_configuration_t {}

#[cfg(feature = "docs-rs")]
#[allow(non_camel_case_types)]
struct bsec_virtual_sensor_t {}

pub mod bme;
pub mod clock;
pub mod error;

static BSEC_IN_USE: AtomicBool = AtomicBool::new(false);

/// Handle to encapsulates the *Bosch BSEC* library and related state.
pub struct Bsec<S: BmeSensor, C: Clock, B: Borrow<C>> {
    bme: S,
    subscribed: u32,
    ulp_plus_queue: u32,
    next_measurement: i64,
    clock: B,
    _clock_type: PhantomData<C>,
}

impl<S: BmeSensor, C: Clock, B: Borrow<C>> Bsec<S, C, B> {
    /// Initialize the *Bosch BSEC* library and return a handle to interact with
    /// it.
    ///
    /// * `bme`: [`BmeSensor`] implementation to communicate with the BME sensor.
    /// * `clock`: [`Clock`] implementation to obtain timestamps.
    pub fn init(bme: S, clock: B) -> Result<Self, Error<S::Error>> {
        if BSEC_IN_USE
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok()
        {
            unsafe {
                bsec_init().into_result()?;
            }
            Ok(Self {
                bme,
                subscribed: 0,
                ulp_plus_queue: 0,
                next_measurement: clock.borrow().timestamp_ns(),
                clock,
                _clock_type: PhantomData,
            })
        } else {
            Err(Error::BsecAlreadyInUse)
        }
    }

    /// Change subscription to virtual sensor outputs.
    ///
    /// * `requests`: Configuration of virtual sensors and their sample
    ///   rates to subscribe to.
    ///
    /// Returns a vector describing physical sensor and sampling rates required
    /// as input to the BSEC algorithm.
    pub fn update_subscription(
        &mut self,
        requests: &[SubscriptionRequest],
    ) -> Result<Vec<RequiredInput>, Error<S::Error>> {
        let bsec_requested_outputs: Vec<bsec_sensor_configuration_t> =
            requests.iter().map(From::from).collect();
        let mut required_sensor_settings = [bsec_sensor_configuration_t {
            sample_rate: 0.,
            sensor_id: 0,
        }; BSEC_MAX_PHYSICAL_SENSOR as usize];
        let mut n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR as u8;
        unsafe {
            bsec_update_subscription(
                bsec_requested_outputs.as_ptr(),
                requests
                    .len()
                    .try_into()
                    .or(Err(Error::ArgumentListTooLong))?,
                required_sensor_settings.as_mut_ptr(),
                &mut n_required_sensor_settings,
            )
            .into_result()?
        }
        for changed in requests.iter() {
            match changed.sample_rate {
                SampleRate::Disabled => {
                    self.subscribed &= !(changed.sensor as u32);
                    self.ulp_plus_queue &= !(changed.sensor as u32);
                }
                SampleRate::UlpMeasurementOnDemand => {
                    self.ulp_plus_queue |= changed.sensor as u32;
                }
                _ => {
                    self.subscribed |= changed.sensor as u32;
                }
            }
        }
        Ok(required_sensor_settings
            .iter()
            .take(n_required_sensor_settings as usize)
            .map(RequiredInput::from)
            .collect())
    }

    /// Returns the timestamp when the next measurement has to be triggered.
    pub fn next_measurement(&self) -> i64 {
        self.next_measurement
    }

    /// Trigger the next measurement.
    ///
    /// Returns the duration until the measurement becomes available. Call
    /// [`Self::process_last_measurement`] after the duration has passed.
    pub fn start_next_measurement(&mut self) -> nb::Result<Duration, Error<S::Error>> {
        let mut bme_settings = bsec_bme_settings_t {
            next_call: 0,
            process_data: 0,
            heater_temperature: 0,
            heating_duration: 0,
            run_gas: 0,
            pressure_oversampling: 0,
            temperature_oversampling: 0,
            humidity_oversampling: 0,
            trigger_measurement: 0,
        };
        unsafe {
            bsec_sensor_control(self.clock.borrow().timestamp_ns(), &mut bme_settings)
                .into_result()
                .map_err(Error::BsecError)?;
        }
        self.next_measurement = bme_settings.next_call;
        if bme_settings.trigger_measurement != 1 {
            return Err(nb::Error::WouldBlock);
        }
        self.bme
            .start_measurement(&BmeSettingsHandle::new(&bme_settings))
            .map_err(Error::BmeSensorError)
            .map_err(nb::Error::Other)
    }

    /// Process the last triggered measurement.
    ///
    /// Call this method after the duration returned from a call to
    /// [`Self::start_next_measurement`] has passed.
    ///
    /// Returns a vector of virtual sensor outputs calculated by the
    /// *Bosch BSEC* library.
    pub fn process_last_measurement(&mut self) -> nb::Result<Vec<Output>, Error<S::Error>> {
        let time_stamp = self.clock.borrow().timestamp_ns();
        let inputs: Vec<bsec_input_t> = self
            .bme
            .get_measurement()
            .map_err(|e| e.map(Error::BmeSensorError))?
            .iter()
            .map(|o| bsec_input_t {
                time_stamp,
                signal: o.signal,
                signal_dimensions: 1,
                sensor_id: o.sensor.into(),
            })
            .collect();
        let mut outputs = vec![
            bsec_output_t {
                time_stamp: 0,
                signal: 0.,
                signal_dimensions: 1,
                sensor_id: 0,
                accuracy: 0,
            };
            (self.subscribed | self.ulp_plus_queue).count_ones() as usize
        ];
        let mut num_outputs: u8 = outputs
            .len()
            .try_into()
            .or(Err(Error::ArgumentListTooLong))?;
        self.ulp_plus_queue = 0;
        unsafe {
            bsec_do_steps(
                inputs.as_ptr(),
                inputs
                    .len()
                    .try_into()
                    .or(Err(Error::ArgumentListTooLong))?,
                outputs.as_mut_ptr(),
                &mut num_outputs,
            )
            .into_result()
            .map_err(Error::BsecError)?;
        }

        let signals: Result<Vec<Output>, Error<S::Error>> = outputs
            .iter()
            .take(num_outputs.into())
            .map(|x| Output::try_from(x).map_err(Error::<S::Error>::from))
            .collect();
        Ok(signals?)
    }

    /// Get the current raw *Bosch BSEC* state, e.g. to persist it before
    /// shutdown.
    pub fn get_state(&self) -> Result<Vec<u8>, Error<S::Error>> {
        let mut state = [0u8; BSEC_MAX_STATE_BLOB_SIZE as usize];
        let mut work_buffer = [0u8; BSEC_MAX_WORKBUFFER_SIZE as usize];
        let mut state_length = BSEC_MAX_STATE_BLOB_SIZE;
        unsafe {
            bsec_get_state(
                0,
                state.as_mut_ptr(),
                state.len() as u32,
                work_buffer.as_mut_ptr(),
                work_buffer.len() as u32,
                &mut state_length,
            )
            .into_result()?;
        }
        Ok(state[..state_length as usize].into())
    }

    /// Set the raw *Bosch BSEC* state, e.g. to restore persisted state after
    /// shutdown.
    pub fn set_state(&mut self, state: &[u8]) -> Result<(), Error<S::Error>> {
        let mut work_buffer = [0u8; BSEC_MAX_WORKBUFFER_SIZE as usize];
        unsafe {
            bsec_set_state(
                state.as_ptr(),
                state.len() as u32,
                work_buffer.as_mut_ptr(),
                work_buffer.len() as u32,
            )
            .into_result()?;
        }
        Ok(())
    }

    /// Get the current (raw) *Bosch BSEC* configuration.
    pub fn get_configuration(&self) -> Result<Vec<u8>, Error<S::Error>> {
        let mut serialized_settings = [0u8; BSEC_MAX_PROPERTY_BLOB_SIZE as usize];
        let mut serialized_settings_length = 0u32;
        let mut work_buffer = [0u8; BSEC_MAX_WORKBUFFER_SIZE as usize];
        unsafe {
            bsec_get_configuration(
                0,
                serialized_settings.as_mut_ptr(),
                serialized_settings.len() as u32,
                work_buffer.as_mut_ptr(),
                work_buffer.len() as u32,
                &mut serialized_settings_length,
            )
            .into_result()?;
        }
        Ok(serialized_settings[..serialized_settings_length as usize].into())
    }

    /// Set the (raw) *Bosch BSEC* configuration.
    ///
    /// Your copy of the *Bosch BSEC* library should contain several different
    /// configuration files. See the Bosch BSEC documentation for more
    /// information.
    pub fn set_configuration(&mut self, serialized_settings: &[u8]) -> Result<(), Error<S::Error>> {
        let mut work_buffer = [0u8; BSEC_MAX_WORKBUFFER_SIZE as usize];
        unsafe {
            bsec_set_configuration(
                serialized_settings.as_ptr(),
                serialized_settings.len() as u32,
                work_buffer.as_mut_ptr(),
                work_buffer.len() as u32,
            )
            .into_result()?
        }
        Ok(())
    }

    /// See documentation of `bsec_reset_output` in the *Bosch BSEC*
    /// documentation.
    pub fn reset_output(&mut self, sensor: OutputKind) -> Result<(), Error<S::Error>> {
        unsafe {
            bsec_reset_output(bsec_virtual_sensor_t::from(sensor) as u8).into_result()?;
        }
        Ok(())
    }
}

impl<S: BmeSensor, C: Clock, B: Borrow<C>> Drop for Bsec<S, C, B> {
    fn drop(&mut self) {
        BSEC_IN_USE.store(false, Ordering::SeqCst);
    }
}

/// Return the *Bosch BSEC* version.
///
/// The returned tuple consists of *major*, *minor*, *major bugfix*, and
/// *minor bugfix* version.
pub fn get_version() -> Result<(u8, u8, u8, u8), BsecError> {
    let mut version = bsec_version_t {
        major: 0,
        minor: 0,
        major_bugfix: 0,
        minor_bugfix: 0,
    };
    unsafe {
        bsec_get_version(&mut version).into_result()?;
    }
    Ok((
        version.major,
        version.minor,
        version.major_bugfix,
        version.minor_bugfix,
    ))
}

/// Encapsulates data read from a BME physical sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Input {
    /// The sensor value read.
    pub signal: f32,

    /// The sensor read.
    pub sensor: InputKind,
}

/// Single virtual sensor output of the BSEC algorithm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Output {
    /// Timestamp (nanoseconds) of the measurement.
    ///
    /// This timestamp is based on the [`Clock`] instance used by [`Bsec`].
    pub timestamp_ns: i64,

    /// Signal value of the virtual sensor.
    pub signal: f64,

    /// Type of virtual sensor.
    pub sensor: OutputKind,

    /// Accuracy of the virtual sensor.
    pub accuracy: Accuracy,
}

impl TryFrom<&bsec_output_t> for Output {
    type Error = ConversionError;
    fn try_from(output: &bsec_output_t) -> Result<Self, ConversionError> {
        Ok(Self {
            timestamp_ns: output.time_stamp,
            signal: output.signal.into(),
            sensor: output.sensor_id.try_into()?,
            accuracy: output.accuracy.try_into()?,
        })
    }
}

/// Sensor accuracy level.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Accuracy {
    Unreliable = 0,
    LowAccuracy = 1,
    MediumAccuracy = 2,
    HighAccuracy = 3,
}

impl TryFrom<u8> for Accuracy {
    type Error = ConversionError;
    fn try_from(accuracy: u8) -> Result<Self, ConversionError> {
        use Accuracy::*;
        match accuracy {
            0 => Ok(Unreliable),
            1 => Ok(LowAccuracy),
            2 => Ok(MediumAccuracy),
            3 => Ok(HighAccuracy),
            _ => Err(ConversionError::InvalidAccuracy(accuracy)),
        }
    }
}

/// Describes a virtual sensor output to request from the *Bosch BSEC* library.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubscriptionRequest {
    /// Desired sample rate of the virtual sensor output.
    pub sample_rate: SampleRate,
    /// Desired virtual output to sample.
    pub sensor: OutputKind,
}

impl From<&SubscriptionRequest> for bsec_sensor_configuration_t {
    fn from(sensor_configuration: &SubscriptionRequest) -> Self {
        Self {
            sample_rate: sensor_configuration.sample_rate.into(),
            sensor_id: bsec_virtual_sensor_t::from(sensor_configuration.sensor) as u8,
        }
    }
}

/// Describes a physical BME sensor that needs to be sampled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RequiredInput {
    /// Sample rate
    pub sample_rate: f32,
    /// Sensor that needs to be sampled
    pub sensor: InputKind,
}

impl From<&bsec_sensor_configuration_t> for RequiredInput {
    fn from(sensor_configuration: &bsec_sensor_configuration_t) -> Self {
        Self {
            sample_rate: sensor_configuration.sample_rate,
            sensor: InputKind::from(sensor_configuration.sensor_id),
        }
    }
}

/// Valid sampling rates for the BSEC algorithm.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SampleRate {
    /// Disabled, not being sampled.
    Disabled,
    /// Ultra-low power, see *Bosch BSEC* documentation.
    Ulp,
    /// Continuous mode for testing, see *Bosch BSEC* documentation.
    Continuous,
    /// Low power, see *Bosch BSEC* documentation.
    Lp,
    /// Perform a single measurement on demand between sampling intervals.
    ///
    /// See *Bosch BSEC* documentation.
    UlpMeasurementOnDemand,
}

impl From<SampleRate> for f32 {
    fn from(sample_rate: SampleRate) -> Self {
        f64::from(sample_rate) as f32
    }
}

impl From<SampleRate> for f64 {
    fn from(sample_rate: SampleRate) -> Self {
        use libalgobsec_sys::*;
        use SampleRate::*;
        match sample_rate {
            Disabled => BSEC_SAMPLE_RATE_DISABLED,
            Ulp => BSEC_SAMPLE_RATE_ULP,
            Continuous => BSEC_SAMPLE_RATE_CONTINUOUS,
            Lp => BSEC_SAMPLE_RATE_LP,
            UlpMeasurementOnDemand => BSEC_SAMPLE_RATE_ULP_MEASUREMENT_ON_DEMAND,
        }
    }
}

/// Identifies a physical BME sensor.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum InputKind {
    /// Pressure sensor.
    Pressure,
    /// Humidity sensor.
    Humidity,
    /// Temperature sensor.
    Temperature,
    /// Gas resistance sensor.
    GasResistor,
    /// Compensation for nearby heat sources.
    HeatSource,
    /// Pseudo-sensor to disable the baseline tracker.
    DisableBaselineTracker,
    /// Other sensor only known by magic number.
    Other(u32),
}

impl From<u8> for InputKind {
    fn from(physical_sensor: u8) -> Self {
        Self::from(physical_sensor as u32)
    }
}

impl From<u32> for InputKind {
    fn from(physical_sensor: u32) -> Self {
        #![allow(non_upper_case_globals)]
        use libalgobsec_sys::*;
        use InputKind::*;
        match physical_sensor {
            bsec_physical_sensor_t_BSEC_INPUT_PRESSURE => Pressure,
            bsec_physical_sensor_t_BSEC_INPUT_HUMIDITY => Humidity,
            bsec_physical_sensor_t_BSEC_INPUT_TEMPERATURE => Temperature,
            bsec_physical_sensor_t_BSEC_INPUT_GASRESISTOR => GasResistor,
            bsec_physical_sensor_t_BSEC_INPUT_HEATSOURCE => HeatSource,
            bsec_physical_sensor_t_BSEC_INPUT_DISABLE_BASELINE_TRACKER => DisableBaselineTracker,
            physical_sensor => Other(physical_sensor),
        }
    }
}

impl From<InputKind> for bsec_physical_sensor_t {
    fn from(physical_sensor: InputKind) -> Self {
        use libalgobsec_sys::*;
        use InputKind::*;
        match physical_sensor {
            Pressure => bsec_physical_sensor_t_BSEC_INPUT_PRESSURE,
            Humidity => bsec_physical_sensor_t_BSEC_INPUT_HUMIDITY,
            Temperature => bsec_physical_sensor_t_BSEC_INPUT_TEMPERATURE,
            GasResistor => bsec_physical_sensor_t_BSEC_INPUT_GASRESISTOR,
            HeatSource => bsec_physical_sensor_t_BSEC_INPUT_HEATSOURCE,
            DisableBaselineTracker => bsec_physical_sensor_t_BSEC_INPUT_DISABLE_BASELINE_TRACKER,
            Other(sensor) => sensor,
        }
    }
}

impl From<InputKind> for u8 {
    fn from(physical_sensor: InputKind) -> Self {
        bsec_physical_sensor_t::from(physical_sensor) as Self
    }
}

/// *Bosch BSEC* virtual sensor output.
///
/// See *Bosch BSEC* documentation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum OutputKind {
    Iaq = 0x0001,
    StaticIaq = 0x0002,
    Co2Equivalent = 0x0004,
    BreathVocEquivalent = 0x0008,
    RawTemperature = 0x0010,
    RawPressure = 0x0020,
    RawHumidity = 0x0040,
    RawGas = 0x0080,
    StabilizationStatus = 0x0100,
    RunInStatus = 0x0200,
    SensorHeatCompensatedTemperature = 0x0400,
    SensorHeatCompensatedHumidity = 0x0800,
    DebugCompensatedGas = 0x1000,
    GasPercentage = 0x2000,
}

impl From<OutputKind> for bsec_virtual_sensor_t {
    fn from(virtual_sensor: OutputKind) -> Self {
        use libalgobsec_sys::*;
        use OutputKind::*;
        match virtual_sensor {
            Iaq => bsec_virtual_sensor_t_BSEC_OUTPUT_IAQ,
            StaticIaq => bsec_virtual_sensor_t_BSEC_OUTPUT_STATIC_IAQ,
            Co2Equivalent => bsec_virtual_sensor_t_BSEC_OUTPUT_CO2_EQUIVALENT,
            BreathVocEquivalent => bsec_virtual_sensor_t_BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
            RawTemperature => bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_TEMPERATURE,
            RawPressure => bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_PRESSURE,
            RawHumidity => bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_HUMIDITY,
            RawGas => bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_GAS,
            StabilizationStatus => bsec_virtual_sensor_t_BSEC_OUTPUT_STABILIZATION_STATUS,
            RunInStatus => bsec_virtual_sensor_t_BSEC_OUTPUT_RUN_IN_STATUS,
            SensorHeatCompensatedTemperature => {
                bsec_virtual_sensor_t_BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE
            }
            SensorHeatCompensatedHumidity => {
                bsec_virtual_sensor_t_BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
            }
            DebugCompensatedGas => bsec_virtual_sensor_t_BSEC_OUTPUT_COMPENSATED_GAS,
            GasPercentage => bsec_virtual_sensor_t_BSEC_OUTPUT_GAS_PERCENTAGE,
        }
    }
}

impl TryFrom<bsec_virtual_sensor_t> for OutputKind {
    type Error = ConversionError;
    fn try_from(virtual_sensor: bsec_virtual_sensor_t) -> Result<Self, ConversionError> {
        #![allow(non_upper_case_globals)]
        use libalgobsec_sys::*;
        use OutputKind::*;
        match virtual_sensor {
            bsec_virtual_sensor_t_BSEC_OUTPUT_IAQ => Ok(Iaq),
            bsec_virtual_sensor_t_BSEC_OUTPUT_STATIC_IAQ => Ok(StaticIaq),
            bsec_virtual_sensor_t_BSEC_OUTPUT_CO2_EQUIVALENT => Ok(Co2Equivalent),
            bsec_virtual_sensor_t_BSEC_OUTPUT_BREATH_VOC_EQUIVALENT => Ok(BreathVocEquivalent),
            bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_TEMPERATURE => Ok(RawTemperature),
            bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_PRESSURE => Ok(RawPressure),
            bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_HUMIDITY => Ok(RawHumidity),
            bsec_virtual_sensor_t_BSEC_OUTPUT_RAW_GAS => Ok(RawGas),
            bsec_virtual_sensor_t_BSEC_OUTPUT_STABILIZATION_STATUS => Ok(StabilizationStatus),
            bsec_virtual_sensor_t_BSEC_OUTPUT_RUN_IN_STATUS => Ok(RunInStatus),
            bsec_virtual_sensor_t_BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE => {
                Ok(SensorHeatCompensatedTemperature)
            }
            bsec_virtual_sensor_t_BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY => {
                Ok(SensorHeatCompensatedHumidity)
            }
            bsec_virtual_sensor_t_BSEC_OUTPUT_COMPENSATED_GAS => Ok(DebugCompensatedGas),
            bsec_virtual_sensor_t_BSEC_OUTPUT_GAS_PERCENTAGE => Ok(GasPercentage),
            _ => Err(ConversionError::InvalidVirtualSensorId(virtual_sensor)),
        }
    }
}

impl TryFrom<u8> for OutputKind {
    type Error = ConversionError;
    fn try_from(virtual_sensor: u8) -> Result<Self, ConversionError> {
        Self::try_from(virtual_sensor as bsec_virtual_sensor_t)
    }
}

trait IntoResult {
    fn into_result(self) -> Result<(), BsecError>;
}

impl IntoResult for bsec_library_return_t {
    fn into_result(self) -> Result<(), BsecError> {
        #![allow(non_upper_case_globals)]
        match self {
            libalgobsec_sys::bsec_library_return_t_BSEC_OK => Ok(()),
            error_code => Err(BsecError::from(error_code)),
        }
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::bme::test_support::FakeBmeSensor;
    use crate::clock::test_support::FakeClock;
    use serial_test::serial;
    use std::collections::HashMap;

    #[test]
    #[serial]
    fn cannot_create_mulitple_bsec_at_the_same_time() {
        let clock = FakeClock::default();
        let first: Bsec<_, FakeClock, _> = Bsec::init(FakeBmeSensor::default(), &clock).unwrap();
        assert!(Bsec::<_, FakeClock, _>::init(FakeBmeSensor::default(), &clock).is_err());
        drop(first);
        let _another = Bsec::<_, FakeClock, _>::init(FakeBmeSensor::default(), &clock).unwrap();
    }

    #[test]
    #[serial]
    fn basic_bsec_operation_smoke_test() {
        let clock = FakeClock::default();
        let sensor = FakeBmeSensor::new(Ok(vec![
            Input {
                sensor: InputKind::Temperature,
                signal: 22.,
            },
            Input {
                sensor: InputKind::Humidity,
                signal: 40.,
            },
            Input {
                sensor: InputKind::Pressure,
                signal: 1000.,
            },
            Input {
                sensor: InputKind::GasResistor,
                signal: 6000.,
            },
        ]));
        let mut bsec: Bsec<_, FakeClock, _> = Bsec::init(sensor, &clock).unwrap();
        bsec.update_subscription(&[
            SubscriptionRequest {
                sample_rate: SampleRate::Lp,
                sensor: OutputKind::RawTemperature,
            },
            SubscriptionRequest {
                sample_rate: SampleRate::Lp,
                sensor: OutputKind::RawHumidity,
            },
            SubscriptionRequest {
                sample_rate: SampleRate::Lp,
                sensor: OutputKind::RawPressure,
            },
            SubscriptionRequest {
                sample_rate: SampleRate::Lp,
                sensor: OutputKind::RawGas,
            },
        ])
        .unwrap();

        clock.advance_by(bsec.start_next_measurement().unwrap());
        let outputs = bsec.process_last_measurement().unwrap();
        assert!(bsec.next_measurement() >= 3_000_000_000);

        let signals: HashMap<OutputKind, &Output> = outputs.iter().map(|s| (s.sensor, s)).collect();
        assert!(
            (signals.get(&OutputKind::RawTemperature).unwrap().signal - 22.).abs() < f64::EPSILON
        );
        assert!((signals.get(&OutputKind::RawHumidity).unwrap().signal - 40.).abs() < f64::EPSILON);
        assert!(
            (signals.get(&OutputKind::RawPressure).unwrap().signal - 1000.).abs() < f64::EPSILON
        );
        assert!((signals.get(&OutputKind::RawGas).unwrap().signal - 6000.).abs() < f64::EPSILON);
    }

    #[test]
    #[serial]
    fn roundtrip_state_smoke_test() {
        let clock = FakeClock::default();
        let sensor = FakeBmeSensor::default();
        let mut bsec: Bsec<_, FakeClock, _> = Bsec::init(sensor, &clock).unwrap();
        let state = bsec.get_state().unwrap();
        bsec.set_state(&state).unwrap();
    }

    #[test]
    #[serial]
    fn configuration_roundtrip_smoke_test() {
        let clock = FakeClock::default();
        let sensor = FakeBmeSensor::default();
        let mut bsec: Bsec<_, FakeClock, _> = Bsec::init(sensor, &clock).unwrap();
        let config = bsec.get_configuration().unwrap();
        bsec.set_configuration(&config).unwrap();
    }

    #[test]
    fn get_version_smoke_test() {
        let version = get_version().unwrap();
        assert!(version.0 == 1);
        assert!(version.1 >= 4);
        assert!(version.1 > 4 || version.2 >= 8);
    }

    #[test]
    #[serial]
    fn reset_output_smoke_test() {
        let clock = FakeClock::default();
        let sensor = FakeBmeSensor::default();
        let mut bsec: Bsec<_, FakeClock, _> = Bsec::init(sensor, &clock).unwrap();
        bsec.reset_output(OutputKind::Iaq).unwrap();
    }
}
