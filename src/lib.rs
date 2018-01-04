#![no_std]
#![feature(unsize)]

#[macro_use]
extern crate bitflags;
extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
extern crate static_ref;

pub extern crate stm32f7x;

pub mod gpio;
pub mod serial;
