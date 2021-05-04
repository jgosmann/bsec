# bsec crate

Rust API to the
[Bosch BSEC library](https://www.bosch-sensortec.com/software-tools/software/bsec/).

This readme will use *bsec* to refer to this crate, while
*Bosch BSEC* is used to refer to the original BSEC library provided by
Bosch.


## Important license information

The *Bosch BSEC* library is proprietary. Thus, the *Bosch BSEC* library and
its documentation cannot be included in the *bsec* Rust crate and need to be
obtained separately.

While the *bsec* documentation covers the Rust crate itself, you will likely
have to refer to the *Bosch BSEC* documentation at some points to get a full
understanding.

You are responsible for adhering to the Bosch BSEC lincese terms in your
products, despite the Rust API in this crate being published under a
permissive license.

* [Bosch BSEC website to obtain your copy](https://www.bosch-sensortec.com/software-tools/software/bsec/)
* [Bosch BESC license terms at the time of writing](https://www.bosch-sensortec.com/media/boschsensortec/downloads/bsec/2017-07-17_clickthrough_license_terms_environmentalib_sw_clean.pdf)


## Features

* Safe Rust API bindings to the *Bosch BSEC* library.
* Implementation to use it with the
  [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
  sensor.
* Extensible to other sensors.
* Rudimentary fake sensor implementation to use in unit tests.


## Documentation

[The crate documentation can be found on docs.rs.](https://docs.rs/bsec/latest/bsec/index.html)


## Getting started

[See the crate documentation.](https://docs.rs/bsec/latest/bsec/index.html#getting-started)


## Usage example

```rust
use bsec::{Bsec, Input, InputKind, OutputKind, clock::Clock, SampleRate, SubscriptionRequest};
use nb::block;
use std::time::Duration;

// Acquire handle to the BSEC library.
// Only one such handle can be acquired at any time.
let mut bsec: Bsec<_, TimePassed, _> = Bsec::init(sensor, &clock)?;

// Configure the outputs you want to subscribe to.
bsec.update_subscription(&[
    SubscriptionRequest {
        sample_rate: SampleRate::Lp,
        sensor: OutputKind::Iaq,
    },
])?;

// We need to feed BSEC regularly with new measurements.
loop {
    // Wait for when the next measurement is due.
    sleep_for(Duration::from_nanos((bsec.next_measurement() - clock.timestamp_ns()) as u64));

    // Start the measurement.
    let wait_duration = block!(bsec.start_next_measurement())?;
    sleep_for(wait_duration);
    # clock.advance_by(wait_duration);

    // Process the measurement when ready and print the BSEC outputs.
    let outputs = block!(bsec.process_last_measurement())?;
    for output in &outputs {
        println!("{:?}: {}", output.sensor, output.signal);
    }
}
```