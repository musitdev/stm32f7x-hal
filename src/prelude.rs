//! The prelude is a collection of all the traits in this crate
//!
//! The traits have been renamed to avoid collisions with other items when
//! performing a glob import.

pub use gpio::GpioExt as _stm32f7x_hal_gpio_GpioExt;
pub use hal::prelude::*;
pub use rcc::RccExt as _stm32f7x_hal_rcc_RccExt;
pub use time::U32Ext as _stm32f7x_hal_time_U32Ext;
pub use flash::FlashExt as _stm32f7x_hal_flash_FlashExt;
