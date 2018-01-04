//! Serial interface
//!
//! You can use the `Serial` interface with these USART instances
//!
//! # USART1
//!
//! - TX = PA9
//! - RX = PA10
//! - Interrupt = USART1
//!
//! # USART2
//!
//! - TX = PA2
//! - RX = PA3
//! - Interrupt = USART2
//!
//! # USART3
//!
//! - TX = PB10
//! - RX = PB11
//! - Interrupt = USART3

use core::any::{Any, TypeId};
use core::marker::Unsize;
use core::ops::Deref;
use core::ptr;

use cast::u16;
use hal;
use nb;
use static_ref::Static;
use stm32f7x::{gpioa, DMA1, USART1, USART2, USART3, GPIOA,
                  GPIOB, RCC};

/// An error
#[derive(Debug)]
pub enum Error {
    /// De-synchronization, excessive noise or a break character detected
    Framing,
    /// Noise detected in the received frame
    Noise,
    /// RX buffer overrun
    Overrun,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// RX buffer Not Empty (new data available)
    Rxne,
    /// Transmission Complete
    Tc,
    /// TX buffer Empty (more data can be send)
    Txe,
}

/// Specialized `Result` type
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;


/// Usart trait
///
/// Allows the init of Usart
pub trait Usart {
    /// Initializes the serial interface with a baud rate of `baut_rate` bits
    /// per second
    ///
    /// The serial interface will be configured to use 8 bits of data, 1 stop
    /// bit, no hardware control and to omit parity checking
    fn init(
        &self,
        baud_rate: usize,
        afio: &AFIO,
        dma1: Option<&DMA1>,
        gpio: &U::GPIO,
        rcc: &RCC,
    )
    {
        rcc.apb2enr.modify(|_, w| {
            w.afioen().enabled().iopaen().enabled().usart1en().enabled()
        });
        
        // PA9 = TX, PA10 = RX
        afio.mapr.modify(|_, w| w.usart1_remap().clear_bit());
        gpio.crh.modify(|_, w| {
            w.mode9()
                .output()
                .cnf9()
                .alt_push()
                .mode10()
                .input()
                .cnf10()
                .bits(0b01)
        });
        // TX DMA transfer
        // mem2mem: Memory to memory mode disabled
        // pl: Medium priority
        // msize: Memory size = 8 bits
        // psize: Peripheral size = 8 bits
        // minc: Memory increment mode enabled
        // pinc: Peripheral increment mode disabled
        // circ: Circular mode disabled
        // dir: Transfer from memory to peripheral
        // tceie: Transfer complete interrupt enabled
        // en: Disabled
        dma1.ccr4.write(|w| unsafe {
            w.mem2mem()
                .clear_bit()
                .pl()
                .bits(0b01)
                .msize()
                .bits(0b00)
                .psize()
                .bits(0b00)
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .pinc()
                .clear_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });

        // RX DMA transfer
        // mem2mem: Memory to memory mode disabled
        // pl: Medium priority
        // msize: Memory size = 8 bits
        // psize: Peripheral size = 8 bits
        // minc: Memory increment mode enabled
        // pinc: Peripheral increment mode disabled
        // circ: Circular mode disabled
        // dir: Transfer from peripheral to memory
        // tceie: Transfer complete interrupt enabled
        // en: Disabled
        dma1.ccr5.write(|w| unsafe {
            w.mem2mem()
                .clear_bit()
                .pl()
                .bits(0b01)
                .msize()
                .bits(0b00)
                .psize()
                .bits(0b00)
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .pinc()
                .clear_bit()
                .dir()
                .clear_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });

        // 8N1
        usart.cr2.write(|w| unsafe { w.stop().bits(0b00) });

        // baud rate
        let brr = baud_rate.into();

        assert!(brr >= 16, "impossible baud rate");

        usart.brr.write(|w| unsafe { w.bits(brr) });

        // disable hardware flow control
        // enable DMA TX and RX transfers
        usart.cr3.write(|w| {
            w.rtse()
                .clear_bit()
                .ctse()
                .clear_bit()
                .dmat()
                .set_bit()
                .dmar()
                .set_bit()
        });

        // enable TX, RX; disable parity checking
        usart.cr1.write(|w| {
            w.ue()
                .set_bit()
                .re()
                .set_bit()
                .te()
                .set_bit()
                .m()
                .clear_bit()
                .pce()
                .clear_bit()
                .rxneie()
                .clear_bit()
        });

    }
}

