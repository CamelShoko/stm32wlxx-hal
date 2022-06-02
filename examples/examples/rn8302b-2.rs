// RN8302B energy AFE spi communication

#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::unwrap;
use defmt_rtt as _;
use hal::spi;
// global logger
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal,
    cortex_m::{self, delay::Delay},
    embedded_hal::blocking::spi::Transfer,
    gpio::{pins, Output, PinState, PortA, PortB},
    pac,
    pwr::enable_shutdown_sleeponexit,
    spi::{BaudRate::Div64, Spi, MODE_1},
    uart::{self, LpUart},
    util::new_delay,
};
struct Rn8302 {
    bus: spi::Spi<pac::SPI1, pins::A5, pins::A6, pins::A7>,
    power: Output<pins::B6>,
    chip: Output<pins::A4>,
    delay: Delay,
}

impl Rn8302 {
    fn power_on(&mut self, stable: u32) {
        defmt::info!("power on rn8302-afe");
        self.power.set_level(PinState::High);
        self.delay.delay_ms(stable);
        self.power.set_level(PinState::Low);
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
            output[i] = input[2 + i]
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

unsafe fn rn8302_setup() -> Rn8302 {
    let mut dp: pac::Peripherals = pac::Peripherals::steal();
    let cp: pac::CorePeripherals = pac::CorePeripherals::steal();

    let delay: Delay = new_delay(cp.SYST, &dp.RCC);
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    let chip: Output<pins::A4> = cortex_m::interrupt::free(|cs| (Output::default(gpioa.a4, cs)));
    let power: Output<pins::B6> = cortex_m::interrupt::free(|cs| (Output::default(gpiob.b6, cs)));

    let bus = cortex_m::interrupt::free(|cs| {
        Spi::new_spi1_full_duplex(
            dp.SPI1,
            (gpioa.a5, gpioa.a6, gpioa.a7),
            MODE_1,
            Div64,
            &mut dp.RCC,
            cs,
        )
    });

    Rn8302 {
        bus,
        power,
        chip,
        delay,
    }
}

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let mut cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());

    // lpuart
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    let mut lpuart: LpUart<pins::A3, pins::A2> = cortex_m::interrupt::free(|cs| {
        LpUart::new(dp.LPUART, 115200, uart::Clk::Hsi16, &mut dp.RCC)
            .enable_rx(gpioa.a3, cs)
            .enable_tx(gpioa.a2, cs)
    });
    unwrap!(write!(&mut lpuart, "rn8302b\r\n").ok());

    // rn8302
    let mut rn8302: Rn8302 = unsafe { rn8302_setup() };
    rn8302.power_on(50);
    let id = rn8302.read_4byte(0x01, 0x8F);
    defmt::info!("rn8302 id ---> {:#04X}", id);
    let mode = rn8302.read_1byte(0x01, 0x81);
    defmt::info!("rn8302 {} mode", rn8302.wmstr(mode));

    loop {
        enable_shutdown_sleeponexit(&mut dp.PWR, &mut cp.SCB);
    }
}
