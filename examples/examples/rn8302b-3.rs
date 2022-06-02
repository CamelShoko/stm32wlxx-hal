// RN8302B energy AFE spi communication

#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::unwrap;
use defmt_rtt as _; // global logger
use hal::spi;
use nb;
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal,
    cortex_m::{self, delay::Delay},
    embedded_hal::blocking::spi::Transfer,
    embedded_hal::prelude::*,
    gpio::{pins, Output, PinState, PortA, PortB},
    pac,
    pwr::enable_shutdown_sleeponexit,
    spi::{BaudRate::Div64, Spi, MODE_1},
    uart::{self, LpUart},
    util::new_delay,
};
struct Rn8302 {
    bus: spi::Spi<pac::SPI1, pins::A5, pins::A6, pins::A7>,
    open: Output<pins::B6>,
    chip: Output<pins::A4>,
    delay: Delay,
}

impl Rn8302 {
    #[inline]
    pub fn new(
        spi: pac::SPI1,
        rcc: &mut pac::RCC,
        syst: pac::SYST,
        sclk: pins::A5,
        miso: pins::A6,
        mosi: pins::A7,
        ness: pins::A4,
        ctrl: pins::B6,
    ) -> Self {
        let delay: Delay = new_delay(syst, rcc);

        let chip: Output<pins::A4> = cortex_m::interrupt::free(|cs| (Output::default(ness, cs)));
        let open: Output<pins::B6> = cortex_m::interrupt::free(|cs| (Output::default(ctrl, cs)));

        let bus = cortex_m::interrupt::free(|cs| {
            Spi::new_spi1_full_duplex(spi, (sclk, miso, mosi), MODE_1, Div64, rcc, cs)
        });

        Rn8302 {
            bus,
            open,
            chip,
            delay,
        }
    }

    fn power_on(&mut self, stable: u32) {
        defmt::info!("turn on rn8302-afe");
        self.open.set_level(PinState::High);
        self.delay.delay_ms(stable);
        self.open.set_level(PinState::Low);
        self.delay.delay_ms(stable);
    }

    fn read_1byte(&mut self, bank: u8, addr: u8) -> u8 {
        let mut input: [u8; 3] = [0; 3];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        defmt::debug!("reg ---> {:#04X}", input);
        let output = input[2];
        output
    }

    fn read_4byte(&mut self, bank: u8, addr: u8) -> [u8; 4] {
        let mut input: [u8; 7] = [0; 7];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        defmt::debug!("reg ---> {:#04X}", input);
        let mut output: [u8; 4] = [0; 4];
        for i in 0..4 {
            output[i] = input[2 + i];
        }
        output
    }

    const fn wmstr(self, reg: u8) -> &'static str {
        match reg {
            0x01 => "EMM",
            0x03 => "SLM",
            _ => "UNKNOWN",
        }
    }
}

struct Serial {
    bus: LpUart<pins::A3, pins::A2>,
}

impl Serial {
    #[inline]
    pub fn new(lpuart: pac::LPUART, rcc: &mut pac::RCC, rx: pins::A3, tx: pins::A2) -> Self {
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().is_not_ready() {}
        let bus = cortex_m::interrupt::free(|cs| {
            LpUart::new(lpuart, 115200, uart::Clk::Hsi16, rcc)
                .enable_rx(rx, cs)
                .enable_tx(tx, cs)
        });
        defmt::info!("serial bus init done");
        Serial { bus }
    }

    fn send_string(&mut self, s: &str) {
        defmt::info!("input string {}", s);
        unwrap!(write!(self.bus, "{}", s).ok());
    }

    fn send_hex(&mut self, hex: &[u8]) {
        defmt::info!("input hex {:#04X}", hex);
        for byte in hex.into_iter() {
            unwrap!(nb::block!(self.bus.write(*byte)));
        }
    }
}

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let mut cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);

    // serial
    let mut serial: Serial = Serial::new(dp.LPUART, &mut dp.RCC, gpioa.a3, gpioa.a2);
    serial.send_string("rn8302\r\n");
    let hex: [u8; 4] = [0x30, 0x31, 0x32, 0x33];
    serial.send_hex(&hex);

    // rn8302
    let mut rn8302: Rn8302 = Rn8302::new(
        dp.SPI1,
        &mut dp.RCC,
        cp.SYST,
        gpioa.a5,
        gpioa.a6,
        gpioa.a7,
        gpioa.a4,
        gpiob.b6,
    );
    rn8302.power_on(50);
    let id = rn8302.read_4byte(0x01, 0x8F);
    defmt::info!("rn8302 id ---> {:#04X}", id);
    let mode = rn8302.read_1byte(0x01, 0x81);
    defmt::info!("rn8302 {} mode", rn8302.wmstr(mode));

    loop {
        enable_shutdown_sleeponexit(&mut dp.PWR, &mut cp.SCB);
    }
}
