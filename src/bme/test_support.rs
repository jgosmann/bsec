//! Provides a fake [`BmeSensor`] implementation to be used in unit tests.
//!
//! This module is only available if the **test-support** feature is enabled.

use super::*;

/// "Unit" error type with only a single variant.
#[derive(Copy, Clone, Debug)]
pub struct UnitError;

impl std::error::Error for UnitError {}

impl std::fmt::Display for UnitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        f.write_fmt(format_args!("{:?}", self))
    }
}

/// Fake [`BmeSensor`] implementation.
///
/// Stores a single constant measurement that will always be returned.
pub struct FakeBmeSensor {
    measurement: nb::Result<Vec<Input>, UnitError>,
}

impl FakeBmeSensor {
    /// Create a new [`FakeBmeSensor`] always returning `measurement`.
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
