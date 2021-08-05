//! General purpose input-output pins

use crate::{adc, pac};
use core::ptr::{read_volatile, write_volatile};
use cortex_m::interrupt::CriticalSection;

/// GPIO output types.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum OutputType {
    /// Push-pull output.
    PushPull = 0b0,
    /// Open-drain output.
    ///
    /// This is typically used with [`Pull::Up`].
    OpenDrain = 0b1,
}

/// GPIO speeds.
///
/// Refer to the device datasheet for the frequency specifications and the power
/// supply and load conditions for each speed.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum Speed {
    Low = 0b00,
    Medium = 0b01,
    Fast = 0b10,
    High = 0b11,
}

/// GPIO pull-up and pull-down.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[non_exhaustive] // maybe they will use that reserved value one day...
pub enum Pull {
    /// No pull-up, no pull-down.
    None = 0b00,
    /// Pull-up.
    Up = 0b01,
    /// Pull-down.
    Down = 0b10,
}

#[derive(Debug)]
struct Pin<const BASE: usize, const N: u8> {}

impl<const BASE: usize, const N: u8> Pin<BASE, N> {
    const MODER_R: *const u32 = BASE as *const u32;
    const MODER_W: *mut u32 = BASE as *mut u32;
    const OTYPER_R: *const u32 = (BASE + 0x4) as *const u32;
    const OTYPER_W: *mut u32 = (BASE + 0x4) as *mut u32;
    const OSPEEDR_R: *const u32 = (BASE + 0x8) as *const u32;
    const OSPEEDR_W: *mut u32 = (BASE + 0x8) as *mut u32;
    const PUPDR_R: *const u32 = (BASE + 0xC) as *const u32;
    const PUPDR_W: *mut u32 = (BASE + 0xC) as *mut u32;
    const IDR: *const u32 = (BASE + 0x10) as *const u32;
    const ODR: *const u32 = (BASE + 0x14) as *const u32;
    const BSRR: *mut u32 = (BASE + 0x18) as *mut u32;

    const AF: usize = if N > 7 { BASE + 0x24 } else { BASE + 0x20 };
    const AF_R: *const u32 = Self::AF as *const u32;
    const AF_W: *mut u32 = Self::AF as *mut u32;
    const AF_SHIFT: u8 = if N > 7 { (N - 7) * 4 } else { N * 4 };

    pub(crate) const fn new() -> Pin<BASE, N> {
        Pin {}
    }

    #[inline(always)]
    pub(crate) unsafe fn set_mode(&mut self, _cs: &CriticalSection, mode: sealed::Mode) {
        let mut val: u32 = read_volatile(Self::MODER_R);
        val &= !(0b11 << (N * 2));
        val |= (mode as u8 as u32) << (N * 2);
        write_volatile(Self::MODER_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_output_type(&mut self, _cs: &CriticalSection, ot: OutputType) {
        let mut val: u32 = read_volatile(Self::OTYPER_R);
        match ot {
            OutputType::PushPull => val &= !(1 << N),
            OutputType::OpenDrain => val |= 1 << N,
        }
        write_volatile(Self::OTYPER_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_speed(&mut self, _cs: &CriticalSection, speed: Speed) {
        let mut val: u32 = read_volatile(Self::OSPEEDR_R);
        val &= !(0b11 << (N * 2));
        val |= (speed as u8 as u32) << (N * 2);
        write_volatile(Self::OSPEEDR_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_pull(&mut self, _cs: &CriticalSection, pull: Pull) {
        let mut val: u32 = read_volatile(Self::PUPDR_R);
        val &= !(0b11 << (N * 2));
        val |= (pull as u8 as u32) << (N * 2);
        write_volatile(Self::PUPDR_W, val);
    }

    #[inline(always)]
    pub(crate) fn input_level(&self) -> Level {
        if unsafe { read_volatile(Self::IDR) } & (1 << N) == 0 {
            Level::Low
        } else {
            Level::High
        }
    }

    #[inline(always)]
    pub(crate) fn output_level(&self) -> Level {
        if unsafe { read_volatile(Self::ODR) } & (1 << N) == 0 {
            Level::Low
        } else {
            Level::High
        }
    }

    #[inline(always)]
    pub(crate) fn set_output_level(&mut self, level: Level) {
        let val: u32 = match level {
            Level::Low => 1 << (N + 16),
            Level::High => 1 << N,
        };
        unsafe { write_volatile(Self::BSRR, val) }
    }

    #[inline(always)]
    pub(crate) fn set_alternate_function(&mut self, af: u8) {
        cortex_m::interrupt::free(|cs| unsafe {
            self.set_mode(cs, sealed::Mode::Alternate);
            let mut val: u32 = read_volatile(Self::AF_R);
            val &= !(0b1111 << Self::AF_SHIFT);
            val |= (af as u8 as u32) << Self::AF_SHIFT;
            write_volatile(Self::AF_W, val);
        })
    }
}

pub(crate) mod sealed {
    use super::{adc, CriticalSection, Level, OutputType, Pull, Speed};

    /// GPIO modes.
    #[repr(u8)]
    pub enum Mode {
        Input = 0b00,
        Output = 0b01,
        Alternate = 0b10,
        Analog = 0b11,
    }

    /// This is the same methods as Pin, but in a trait so that the individual
    /// Pin structures can implement it in a light wrapper without putting a ton
    /// of code into the macro which will result in longer compile times.
    pub trait PinOps {
        unsafe fn set_mode(&mut self, cs: &CriticalSection, mode: Mode);
        unsafe fn set_output_type(&mut self, cs: &CriticalSection, ot: OutputType);
        unsafe fn set_speed(&mut self, cs: &CriticalSection, speed: Speed);
        unsafe fn set_pull(&mut self, cs: &CriticalSection, pull: Pull);
        fn input_level(&self) -> Level;
        fn output_level(&self) -> Level;
        fn set_output_level(&mut self, level: Level);
        fn set_alternate_function(&mut self, af: u8);
    }

    /// Indicate a GPIO pin has the SPI1 MOSI alternate function
    pub trait Spi1Mosi {
        /// Initialize the GPIO pin for use as SPI1 MOSI
        fn set_spi1_mosi_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI1 MISO alternate function
    pub trait Spi1Miso {
        /// Initialize the GPIO pin for use as SPI1 MISO
        fn set_spi1_miso_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI1 SCK alternate function
    pub trait Spi1Sck {
        /// Initialize the GPIO pin for use as SPI1 SCK
        fn set_spi1_sck_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI1 NSS alternate function
    pub trait Spi1Nss {
        /// Initialize the GPIO pin for use as SPI1 NSS
        fn set_spi1_nss_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI2 MOSI alternate function
    pub trait Spi2Mosi {
        /// Initialize the GPIO pin for use as SPI2 MOSI
        fn set_spi2_mosi_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI2 MISO alternate function
    pub trait Spi2Miso {
        /// Initialize the GPIO pin for use as SPI12 MISO
        fn set_spi2_miso_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI2 SCK alternate function
    pub trait Spi2Sck {
        /// Initialize the GPIO pin for use as SPI2 SCK
        fn set_spi2_sck_af(&mut self);
    }

    /// Indicate a GPIO pin has the SPI2 NSS alternate function
    pub trait Spi2Nss {
        /// Initialize the GPIO pin for use as SPI2 NSS
        fn set_spi2_nss_af(&mut self);
    }

    /// Indicate a GPIO pin has the debug SubGHz SPI MOSI alternate function
    pub trait SubGhzSpiMosi {
        /// Initialize the GPIO pin for use as debug SubGHz MOSI
        fn set_subghz_spi_mosi_af(&mut self);
    }

    /// Indicate a GPIO pin has the debug SubGHz SPI MISO alternate function
    pub trait SubGhzSpiMiso {
        /// Initialize the GPIO pin for use as debug SubGHz MISO
        fn set_subghz_spi_miso_af(&mut self);
    }

    /// Indicate a GPIO pin has the debug SubGHz SPI SCK alternate function
    pub trait SubGhzSpiSck {
        /// Initialize the GPIO pin for use as debug SubGHz SCK
        fn set_subghz_spi_sck_af(&mut self);
    }

    /// Indicate a GPIO pin has the debug SubGHz SPI NSS alternate function
    pub trait SubGhzSpiNss {
        /// Initialize the GPIO pin for use as debug SubGHz NSS
        fn set_subghz_spi_nss_af(&mut self);
    }

    /// Indicate a GPIO pin can be sampled by the ADC
    pub trait AdcCh {
        const ADC_CH: adc::Ch;
    }
}

/// GPIO pins
pub mod pins {
    // Switch to this when avaliable on stable
    // https://github.com/rust-lang/rust/issues/51910
    // const GPIOA_BASE: usize = pac::GPIOA::ptr() as *const _ as usize;
    // const GPIOB_BASE: usize = pac::GPIOB::ptr() as *const _ as usize;
    // const GPIOC_BASE: usize = pac::GPIOC::ptr() as *const _ as usize;

    const GPIOA_BASE: usize = 0x4800_0000;
    const GPIOB_BASE: usize = 0x4800_0400;
    const GPIOC_BASE: usize = 0x4800_0800;

    use super::{adc, CriticalSection, Level, OutputType, Pin, Pull, Speed};

    macro_rules! gpio_struct {
        ($name:ident, $base:expr, $n:expr, $doc:expr) => {
            #[doc=$doc]
            #[derive(Debug)]
            pub struct $name {
                pin: Pin<$base, $n>,
            }

            impl $name {
                pub(crate) const fn new() -> Self {
                    $name { pin: Pin::new() }
                }
            }

            impl super::sealed::PinOps for $name {
                #[inline(always)]
                unsafe fn set_mode(&mut self, cs: &CriticalSection, mode: super::sealed::Mode) {
                    self.pin.set_mode(cs, mode)
                }

                #[inline(always)]
                unsafe fn set_output_type(&mut self, cs: &CriticalSection, ot: OutputType) {
                    self.pin.set_output_type(cs, ot)
                }

                #[inline(always)]
                unsafe fn set_speed(&mut self, cs: &CriticalSection, speed: Speed) {
                    self.pin.set_speed(cs, speed)
                }

                #[inline(always)]
                unsafe fn set_pull(&mut self, cs: &CriticalSection, pull: Pull) {
                    self.pin.set_pull(cs, pull)
                }

                #[inline(always)]
                fn input_level(&self) -> Level {
                    self.pin.input_level()
                }

                #[inline(always)]
                fn output_level(&self) -> Level {
                    self.pin.output_level()
                }

                #[inline(always)]
                fn set_output_level(&mut self, level: Level) {
                    self.pin.set_output_level(level)
                }

                #[inline(always)]
                fn set_alternate_function(&mut self, af: u8) {
                    self.pin.set_alternate_function(af)
                }
            }
        };
    }

    gpio_struct!(A0, GPIOA_BASE, 0, "Port A pin 0");
    gpio_struct!(A1, GPIOA_BASE, 1, "Port A pin 1");
    gpio_struct!(A2, GPIOA_BASE, 2, "Port A pin 2");
    gpio_struct!(A3, GPIOA_BASE, 3, "Port A pin 3");
    gpio_struct!(A4, GPIOA_BASE, 4, "Port A pin 4");
    gpio_struct!(A5, GPIOA_BASE, 5, "Port A pin 5");
    gpio_struct!(A6, GPIOA_BASE, 6, "Port A pin 6");
    gpio_struct!(A7, GPIOA_BASE, 7, "Port A pin 7");
    gpio_struct!(A8, GPIOA_BASE, 8, "Port A pin 8");
    gpio_struct!(A9, GPIOA_BASE, 9, "Port A pin 9");
    gpio_struct!(A10, GPIOA_BASE, 10, "Port A pin 10");
    gpio_struct!(A11, GPIOA_BASE, 11, "Port A pin 11");
    gpio_struct!(A12, GPIOA_BASE, 12, "Port A pin 12");
    gpio_struct!(A13, GPIOA_BASE, 13, "Port A pin 13");
    gpio_struct!(A14, GPIOA_BASE, 14, "Port A pin 14");
    gpio_struct!(A15, GPIOA_BASE, 15, "Port A pin 15");

    gpio_struct!(B0, GPIOB_BASE, 0, "Port B pin 0");
    gpio_struct!(B1, GPIOB_BASE, 1, "Port B pin 1");
    gpio_struct!(B2, GPIOB_BASE, 2, "Port B pin 2");
    gpio_struct!(B3, GPIOB_BASE, 3, "Port B pin 3");
    gpio_struct!(B4, GPIOB_BASE, 4, "Port B pin 4");
    gpio_struct!(B5, GPIOB_BASE, 5, "Port B pin 5");
    gpio_struct!(B6, GPIOB_BASE, 6, "Port B pin 6");
    gpio_struct!(B7, GPIOB_BASE, 7, "Port B pin 7");
    gpio_struct!(B8, GPIOB_BASE, 8, "Port B pin 8");
    gpio_struct!(B9, GPIOB_BASE, 9, "Port B pin 9");
    gpio_struct!(B10, GPIOB_BASE, 10, "Port B pin 10");
    gpio_struct!(B11, GPIOB_BASE, 11, "Port B pin 11");
    gpio_struct!(B12, GPIOB_BASE, 12, "Port B pin 12");
    gpio_struct!(B13, GPIOB_BASE, 13, "Port B pin 13");
    gpio_struct!(B14, GPIOB_BASE, 14, "Port B pin 14");
    gpio_struct!(B15, GPIOB_BASE, 15, "Port B pin 15");

    gpio_struct!(C0, GPIOC_BASE, 0, "Port C pin 0");
    gpio_struct!(C1, GPIOC_BASE, 1, "Port C pin 1");
    gpio_struct!(C2, GPIOC_BASE, 2, "Port C pin 2");
    gpio_struct!(C3, GPIOC_BASE, 3, "Port C pin 3");
    gpio_struct!(C4, GPIOC_BASE, 4, "Port C pin 4");
    gpio_struct!(C5, GPIOC_BASE, 5, "Port C pin 5");
    gpio_struct!(C6, GPIOC_BASE, 6, "Port C pin 6");
    gpio_struct!(C13, GPIOC_BASE, 13, "Port C pin 13");
    gpio_struct!(C14, GPIOC_BASE, 14, "Port C pin 14");
    gpio_struct!(C15, GPIOC_BASE, 15, "Port C pin 15");

    impl super::sealed::Spi1Sck for A0 {
        fn set_spi1_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Nss for A4 {
        fn set_spi1_nss_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Sck for A5 {
        fn set_spi1_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Miso for A6 {
        fn set_spi1_miso_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Mosi for A7 {
        fn set_spi1_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Miso for A11 {
        fn set_spi1_miso_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Mosi for A12 {
        fn set_spi1_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Nss for A15 {
        fn set_spi1_nss_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Nss for B2 {
        fn set_spi1_nss_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Sck for B3 {
        fn set_spi1_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Miso for B4 {
        fn set_spi1_miso_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi1Mosi for B5 {
        fn set_spi1_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Mosi for A10 {
        fn set_spi2_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Mosi for B15 {
        fn set_spi2_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Mosi for C1 {
        fn set_spi2_mosi_af(&mut self) {
            self.pin.set_alternate_function(3)
        }
    }

    impl super::sealed::Spi2Mosi for C3 {
        fn set_spi2_mosi_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Miso for A5 {
        fn set_spi2_miso_af(&mut self) {
            self.pin.set_alternate_function(3)
        }
    }

    impl super::sealed::Spi2Miso for B14 {
        fn set_spi2_miso_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Miso for C2 {
        fn set_spi2_miso_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Sck for A8 {
        fn set_spi2_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Sck for A9 {
        fn set_spi2_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Sck for B10 {
        fn set_spi2_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Sck for B13 {
        fn set_spi2_sck_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Nss for A9 {
        fn set_spi2_nss_af(&mut self) {
            self.pin.set_alternate_function(3)
        }
    }

    impl super::sealed::Spi2Nss for B9 {
        fn set_spi2_nss_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::Spi2Nss for B12 {
        fn set_spi2_nss_af(&mut self) {
            self.pin.set_alternate_function(5)
        }
    }

    impl super::sealed::SubGhzSpiNss for A4 {
        fn set_subghz_spi_nss_af(&mut self) {
            self.pin.set_alternate_function(13)
        }
    }

    impl super::sealed::SubGhzSpiSck for A5 {
        fn set_subghz_spi_sck_af(&mut self) {
            self.pin.set_alternate_function(13)
        }
    }

    impl super::sealed::SubGhzSpiMiso for A6 {
        fn set_subghz_spi_miso_af(&mut self) {
            self.pin.set_alternate_function(13)
        }
    }

    impl super::sealed::SubGhzSpiMosi for A7 {
        fn set_subghz_spi_mosi_af(&mut self) {
            self.pin.set_alternate_function(13)
        }
    }

    impl super::sealed::AdcCh for A10 {
        const ADC_CH: adc::Ch = adc::Ch::In6;
    }
    impl super::sealed::AdcCh for A11 {
        const ADC_CH: adc::Ch = adc::Ch::In7;
    }
    impl super::sealed::AdcCh for A12 {
        const ADC_CH: adc::Ch = adc::Ch::In8;
    }
    impl super::sealed::AdcCh for A13 {
        const ADC_CH: adc::Ch = adc::Ch::In9;
    }
    impl super::sealed::AdcCh for A14 {
        const ADC_CH: adc::Ch = adc::Ch::In10;
    }
    impl super::sealed::AdcCh for A15 {
        const ADC_CH: adc::Ch = adc::Ch::In11;
    }
    impl super::sealed::AdcCh for B1 {
        const ADC_CH: adc::Ch = adc::Ch::In5;
    }
    impl super::sealed::AdcCh for B2 {
        const ADC_CH: adc::Ch = adc::Ch::In4;
    }
    impl super::sealed::AdcCh for B3 {
        const ADC_CH: adc::Ch = adc::Ch::In2;
    }
    impl super::sealed::AdcCh for B4 {
        const ADC_CH: adc::Ch = adc::Ch::In3;
    }
    impl super::sealed::AdcCh for B13 {
        const ADC_CH: adc::Ch = adc::Ch::In0;
    }
    impl super::sealed::AdcCh for B14 {
        const ADC_CH: adc::Ch = adc::Ch::In1;
    }
}

/// Port A GPIOs
#[derive(Debug)]
#[allow(missing_docs)]
pub struct PortA {
    pub pa0: pins::A0,
    pub pa1: pins::A1,
    pub pa2: pins::A2,
    pub pa3: pins::A3,
    pub pa4: pins::A4,
    pub pa5: pins::A5,
    pub pa6: pins::A6,
    pub pa7: pins::A7,
    pub pa8: pins::A8,
    pub pa9: pins::A9,
    pub pa10: pins::A10,
    pub pa11: pins::A11,
    pub pa12: pins::A12,
    pub pa13: pins::A13,
    pub pa14: pins::A14,
    pub pa15: pins::A15,
}

impl PortA {
    const GPIOS: PortA = PortA {
        pa0: pins::A0::new(),
        pa1: pins::A1::new(),
        pa2: pins::A2::new(),
        pa3: pins::A3::new(),
        pa4: pins::A4::new(),
        pa5: pins::A5::new(),
        pa6: pins::A6::new(),
        pa7: pins::A7::new(),
        pa8: pins::A8::new(),
        pa9: pins::A9::new(),
        pa10: pins::A10::new(),
        pa11: pins::A11::new(),
        pa12: pins::A12::new(),
        pa13: pins::A13::new(),
        pa14: pins::A14::new(),
        pa15: pins::A15::new(),
    };

    /// Reset GPIO port A and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO A0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let pa0: pins::A0 = gpioa.pa0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpioa: pac::GPIOA, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpioarst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpioarst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port A GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortA;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortA = unsafe { PortA::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortA::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOA clock.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioaen().disabled());
    }

    /// Enable the GPIOA clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioaen().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Port B GPIOs
#[derive(Debug)]
#[allow(missing_docs)]
pub struct PortB {
    pub pb0: pins::B0,
    pub pb1: pins::B1,
    pub pb2: pins::B2,
    pub pb3: pins::B3,
    pub pb4: pins::B4,
    pub pb5: pins::B5,
    pub pb6: pins::B6,
    pub pb7: pins::B7,
    pub pb8: pins::B8,
    pub pb9: pins::B9,
    pub pb10: pins::B10,
    pub pb11: pins::B11,
    pub pb12: pins::B12,
    pub pb13: pins::B13,
    pub pb14: pins::B14,
    pub pb15: pins::B15,
}

impl PortB {
    const GPIOS: PortB = PortB {
        pb0: pins::B0::new(),
        pb1: pins::B1::new(),
        pb2: pins::B2::new(),
        pb3: pins::B3::new(),
        pb4: pins::B4::new(),
        pb5: pins::B5::new(),
        pb6: pins::B6::new(),
        pb7: pins::B7::new(),
        pb8: pins::B8::new(),
        pb9: pins::B9::new(),
        pb10: pins::B10::new(),
        pb11: pins::B11::new(),
        pb12: pins::B12::new(),
        pb13: pins::B13::new(),
        pb14: pins::B14::new(),
        pb15: pins::B15::new(),
    };

    /// Reset GPIO port B and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO B0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let pb0: pins::B0 = gpiob.pb0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpiob: pac::GPIOB, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpiobrst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpiobrst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port B GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortB;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortB = unsafe { PortB::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortB::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOB clock.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioben().disabled());
    }

    /// Enable the GPIOB clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioben().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Port C GPIOs
#[derive(Debug)]
#[allow(missing_docs)]
pub struct PortC {
    pub pc0: pins::C0,
    pub pc1: pins::C1,
    pub pc2: pins::C2,
    pub pc3: pins::C3,
    pub pc4: pins::C4,
    pub pc5: pins::C5,
    pub pc6: pins::C6,
    pub pc13: pins::C13,
    pub pc14: pins::C14,
    pub pc15: pins::C15,
}

impl PortC {
    const GPIOS: PortC = PortC {
        pc0: pins::C0::new(),
        pc1: pins::C1::new(),
        pc2: pins::C2::new(),
        pc3: pins::C3::new(),
        pc4: pins::C4::new(),
        pc5: pins::C5::new(),
        pc6: pins::C6::new(),
        pc13: pins::C13::new(),
        pc14: pins::C14::new(),
        pc15: pins::C15::new(),
    };

    /// Reset GPIO port C and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO C0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pc0: pins::C0 = gpioc.pc0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpioc: pac::GPIOC, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpiocrst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpiocrst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port C GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortC;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortC = unsafe { PortC::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortC::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOC clock.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpiocen().disabled());
    }

    /// Enable the GPIOC clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpiocen().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Digital input or output level.
#[derive(Debug, Eq, PartialEq, PartialOrd, Ord, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// GPIO logic low.
    Low,
    /// GPIO logic high.
    High,
}

impl Level {
    /// Toggle the level.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::High.toggle(), Level::Low);
    /// assert_eq!(Level::Low.toggle(), Level::High);
    /// ```
    pub const fn toggle(self) -> Level {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }

    /// Returns `true` if the level is low.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::Low.is_low(), true);
    /// assert_eq!(Level::High.is_low(), false);
    /// ```
    pub fn is_low(&self) -> bool {
        matches!(self, Self::Low)
    }

    /// Returns `true` if the level is high.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::High.is_high(), true);
    /// assert_eq!(Level::Low.is_high(), false);
    /// ```
    pub fn is_high(&self) -> bool {
        matches!(self, Self::High)
    }
}

/// Output pin arguments.
///
/// Argument of [`Output::new`].
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct OutputArgs {
    /// Output speed.
    pub speed: Speed,
    /// Initial output level.
    pub level: Level,
    /// Output type.
    pub ot: OutputType,
    /// IO pull configuration.
    ///
    /// This is only used if the output type is [`OutputType::OpenDrain`].
    pub pull: Pull,
}

impl OutputArgs {
    /// Create a new `OutputArgs` struct.
    ///
    /// This is the same as `default`, but in a `const` fn.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::OutputArgs;
    ///
    /// assert_eq!(OutputArgs::new(), OutputArgs::default());
    /// ```
    pub const fn new() -> Self {
        OutputArgs {
            speed: Speed::High,
            level: Level::Low,
            ot: OutputType::PushPull,
            pull: Pull::None,
        }
    }
}

impl Default for OutputArgs {
    fn default() -> Self {
        Self::new()
    }
}

/// Output pin.
#[derive(Debug)]
pub struct Output<P> {
    pin: P,
}

impl<P> Output<P>
where
    P: sealed::PinOps,
{
    /// Create a new output pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO port C3, C4, C5 as outputs.
    /// These are the GPIOs for the RF switch on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{self, pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// const OUTPUT_ARGS: gpio::OutputArgs = gpio::OutputArgs {
    ///     level: gpio::Level::Low,
    ///     speed: gpio::Speed::High,
    ///     ot: gpio::OutputType::PushPull,
    ///     pull: gpio::Pull::None,
    /// };
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc3: Output<pins::C3> = Output::new(gpioc.pc3, &OUTPUT_ARGS);
    /// let mut pc4: Output<pins::C4> = Output::new(gpioc.pc4, &OUTPUT_ARGS);
    /// let mut pc5: Output<pins::C5> = Output::new(gpioc.pc5, &OUTPUT_ARGS);
    /// ```
    pub fn new(mut pin: P, args: &OutputArgs) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_output_type(cs, args.ot);
            if args.ot == OutputType::OpenDrain {
                pin.set_pull(cs, args.pull)
            } else {
                pin.set_pull(cs, Pull::None)
            }
            pin.set_speed(cs, args.speed);
            pin.set_output_level(args.level);
            pin.set_mode(cs, sealed::Mode::Output);
        });
        Output { pin }
    }

    /// Create a new output pin from a GPIO using the default settings.
    ///
    /// # Example
    ///
    /// Configure GPIO C0 as an output.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, OutputArgs, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Output<pins::C0> = Output::default(gpioc.pc0);
    /// ```
    pub fn default(pin: P) -> Self {
        Self::new(pin, &OutputArgs::new())
    }

    /// Free the GPIO pin.
    ///
    /// This will reconfigure the GPIO as a floating input.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an output, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pc0_output: Output<pins::C0> = Output::default(gpioc.pc0);
    /// let pc0: pins::C0 = pc0_output.free();
    /// ```
    pub fn free(mut self) -> P {
        cortex_m::interrupt::free(|cs| unsafe {
            self.pin.set_pull(cs, Pull::None);
            self.pin.set_mode(cs, sealed::Mode::Input);
        });
        self.pin
    }

    /// Set the GPIO output level.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Pulse a GPIO pin.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Level, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Output<pins::C0> = Output::default(gpioc.pc0);
    /// pc0.set_level(Level::High);
    /// pc0.set_level(Level::Low);
    /// ```
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_level(level)
    }

    /// Set the GPIO output level high.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Set GPIO C0 high.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Output<pins::C0> = Output::default(gpioc.pc0);
    /// pc0.set_level_high();
    /// ```
    pub fn set_level_high(&mut self) {
        self.set_level(Level::High)
    }

    /// Set the GPIO output level high.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Set GPIO C0 low.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Output<pins::C0> = Output::default(gpioc.pc0);
    /// pc0.set_level_low();
    /// ```
    pub fn set_level_low(&mut self) {
        self.set_level(Level::Low)
    }

    /// Get the current GPIO output level.
    ///
    /// # Example
    ///
    /// Toggle a GPIO pin.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Level, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Output<pins::C0> = Output::default(gpioc.pc0);
    /// pc0.set_level(pc0.level().toggle());
    /// ```
    pub fn level(&self) -> Level {
        self.pin.output_level()
    }
}

impl<P> embedded_hal::digital::v2::OutputPin for Output<P>
where
    P: sealed::PinOps,
{
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_output_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_output_level(Level::High);
        Ok(())
    }
}

/// Input pin
#[derive(Debug)]
pub struct Input<P> {
    pin: P,
}

impl<P> Input<P>
where
    P: sealed::PinOps,
{
    /// Create a new input pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO C6 as an input.
    /// This is the GPIO for button 3 on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc6: Input<pins::C6> = Input::new(gpioc.pc6, Pull::Up);
    /// ```
    pub fn new(mut pin: P, pull: Pull) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_pull(cs, pull);
            pin.set_output_type(cs, OutputType::PushPull);
            pin.set_mode(cs, sealed::Mode::Input);
        });
        Input { pin }
    }

    /// Create a new input pin from a GPIO with default settings.
    ///
    /// # Example
    ///
    /// Configure GPIO C0 as an input.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc0: Input<pins::C0> = Input::default(gpioc.pc0);
    /// ```
    pub fn default(pin: P) -> Self {
        Self::new(pin, Pull::None)
    }

    /// Free the GPIO pin.
    ///
    /// This will reconfigure the GPIO as a floating input.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an input, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pc0_input: Input<pins::C0> = Input::default(gpioc.pc0);
    /// let pc0: pins::C0 = pc0_input.free();
    /// ```
    pub fn free(mut self) -> P {
        cortex_m::interrupt::free(|cs| unsafe {
            self.pin.set_pull(cs, Pull::None);
            // mode is already input
        });
        self.pin
    }

    /// Get the input level.
    ///
    /// # Example
    ///
    /// Get the input level of C6.
    /// This is the GPIO for button 3 on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, Level, PortC, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut pc6: Input<pins::C6> = Input::new(gpioc.pc6, Pull::Up);
    ///
    /// let button_3_is_pressed: bool = pc6.level() == Level::High;
    /// ```
    pub fn level(&self) -> Level {
        self.pin.input_level()
    }
}

/// Analog pin
#[derive(Debug)]
pub struct Analog<P> {
    pin: P,
}

impl<P> Analog<P>
where
    P: sealed::PinOps + sealed::AdcCh,
{
    /// Create a new analog pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO PB14 as an analog pin (ADC_IN1).
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Analog, PortB, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut pb14: Analog<pins::B14> = Analog::new(gpiob.pb14);
    /// ```
    pub fn new(mut pin: P) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_mode(cs, sealed::Mode::Analog);
        });
        Analog { pin }
    }

    /// Free the GPIO pin.
    ///
    /// This will reconfigure the GPIO as a floating input.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an analog pin, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Analog, PortB, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let pb14: Analog<pins::B14> = Analog::new(gpiob.pb14);
    /// let pb14: pins::B14 = pb14.free();
    /// ```
    pub fn free(mut self) -> P {
        cortex_m::interrupt::free(|cs| unsafe {
            self.pin.set_mode(cs, sealed::Mode::Input);
        });
        self.pin
    }
}

impl<P> From<P> for Analog<P>
where
    P: sealed::PinOps + sealed::AdcCh,
{
    fn from(p: P) -> Self {
        Analog::new(p)
    }
}
