//! HAL for the STM32F7xx family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32F7xx family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://github.com/japaric/embedded-hal
//!
//! # Usage
//!
//! To build applications (binary crates) using this crate follow the [cortex-m-quickstart]
//! instructions and add this crate as a dependency in step number 5 and make sure you enable the
//! "rt" Cargo feature of this crate.
//!
//! [cortex-m-quickstart]: https://docs.rs/cortex-m-quickstart/~0.2.3
//!
//! # Examples
//!
//! Examples of *using* these abstractions can be found in the documentation of the [`stm32f7x-hal-example`] crate.
//!
//! [`stm32f7x-hal-example`]: https://docs.rs/stm32f7x-hal-example

#![no_std]
#![feature(never_type)]

extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32f7x;

pub mod rcc;
pub mod time;
pub mod flash;

pub mod gpio;
pub mod serial;
pub mod timer;
pub mod spi;
pub mod i2c;
pub mod delay;
pub mod prelude;