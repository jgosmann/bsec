//! Interface to measure time.

use std::time::Instant;

/// The BSEC algorithem needs a clock capable of providing timestamps.
/// Implement this trait according to your hardware platform.
///
/// # Example
///
/// A possible implementation based on [`std::time`] could look like this:
///
/// ```
/// use bsec::clock::Clock;
/// use std::time::{Duration, Instant};
///
/// pub struct TimePassed {
///     start: Instant,
/// }
///
/// impl TimePassed {
///     fn new() -> Self {
///         Self { start: Instant::now() }
///     }
/// }
///
/// impl Clock for TimePassed {
///     fn timestamp_ns(&self) -> i64 {
///         self.start.elapsed().as_nanos() as i64
///     }
/// }
/// ```
pub trait Clock {
    /// Return a monotonically increasing timestamp with nanosecond resolution.
    ///
    /// The reference point may be arbitrary.
    fn timestamp_ns(&self) -> i64;
}

/// Measures time since the creation of an instance.
pub struct TimePassed {
    start: Instant,
}

impl TimePassed {
    /// Create a new instance starting time measurement at instant of
    /// instantiation.
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Default for TimePassed {
    fn default() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Clock for TimePassed {
    fn timestamp_ns(&self) -> i64 {
        self.start.elapsed().as_nanos() as i64
    }
}

#[cfg(test)]
pub mod tests {
    use super::{Clock, TimePassed};

    #[test]
    fn time_passed_smoke_test() {
        assert!(TimePassed::default().timestamp_ns() >= 0);
    }
}

/// Provides a fake [`Clock`] implementation to be used in unit tests.
///
/// This module is only available if the **test-support** feature is enabled.
#[cfg(any(test, feature = "test-support"))]
pub mod test_support {
    use std::{cell::RefCell, time::Duration};

    use super::*;

    /// A fake [`Clock`] for unit tests that can be advanced manually.
    ///
    /// Each call to [`FakeClock::timestamp_ns`] will advance by one nanosecond.
    ///
    /// # Example
    ///
    /// ```
    /// # use bsec::clock::{Clock, test_support::FakeClock};
    /// let clock = FakeClock::new();
    /// assert_eq!(clock.timestamp_ns(), 1);
    /// assert_eq!(clock.timestamp_ns(), 2);
    /// assert_eq!(clock.timestamp_ns(), 3);
    /// clock.advance_by(std::time::Duration::from_nanos(5));
    /// assert_eq!(clock.timestamp_ns(), 9);
    /// ```
    #[derive(Default)]
    pub struct FakeClock {
        timestamp_ns: RefCell<i64>,
    }

    impl FakeClock {
        /// Create a new [`FakeClock`] initialized with a zero timestamp.
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Clock for FakeClock {
        /// Returns the current timestamp and advances it by one.
        fn timestamp_ns(&self) -> i64 {
            *self.timestamp_ns.borrow_mut() += 1;
            *self.timestamp_ns.borrow()
        }
    }

    impl FakeClock {
        /// Advance the clock's internal time by `duration`.
        pub fn advance_by(&self, duration: Duration) {
            *self.timestamp_ns.borrow_mut() += duration.as_nanos() as i64;
        }
    }
}
