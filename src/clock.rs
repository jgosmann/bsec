//! TODO

/// The bsec crate needs a clock capable of providing nanosecond timestamps.
/// Implement this trait according to your hardware platform.
pub trait Clock {
    /// Return a monotonically increasing timestamp with nanosecond resolution.
    ///
    /// The reference point may be arbitrary.
    fn timestamp_ns(&self) -> i64;
}

#[cfg(any(test, feature = "test_support"))]
pub mod test_support {
    use std::{cell::RefCell, time::Duration};

    use super::*;

    #[derive(Default)]
    pub struct FakeClock {
        timestamp_ns: RefCell<i64>,
    }

    impl Clock for FakeClock {
        fn timestamp_ns(&self) -> i64 {
            *self.timestamp_ns.borrow_mut() += 1;
            *self.timestamp_ns.borrow()
        }
    }

    impl FakeClock {
        pub fn advance_by(&self, duration: Duration) {
            *self.timestamp_ns.borrow_mut() += duration.as_nanos() as i64;
        }
    }
}
