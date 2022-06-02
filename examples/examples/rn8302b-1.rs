// RN8302B energy AFE spi communication

#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::unwrap;
use defmt_rtt as _; // global logger
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

const fn rn8302_wmstr(reg: u8) -> &'static str {
    match reg {
        0x01 => "EMM",
        0x03 => "SLM",
        _ => "UNKNOWN",
    }
}

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let mut cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());

    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    let mut delay: Delay = new_delay(cp.SYST, &dp.RCC);

    // lpuart
    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    let mut lpuart: LpUart<pins::A3, pins::A2> = cortex_m::interrupt::free(|cs| {
        LpUart::new(dp.LPUART, 115200, uart::Clk::Hsi16, &mut dp.RCC)
            .enable_rx(gpioa.a3, cs)
            .enable_tx(gpioa.a2, cs)
    });
    unwrap!(write!(&mut lpuart, "rn8302b spi\r\n").ok());

    // spi
    let mut rn8302_cs: Output<pins::A4> =
        cortex_m::interrupt::free(|cs| (Output::default(gpioa.a4, cs)));
    let mut rn8302_power: Output<pins::B6> =
        cortex_m::interrupt::free(|cs| (Output::default(gpiob.b6, cs)));
    defmt::info!("power on rn8302");
    rn8302_power.set_level(PinState::High);
    delay.delay_ms(50);
    rn8302_power.set_level(PinState::Low);
    delay.delay_ms(50);

    let mut rn8302 = cortex_m::interrupt::free(|cs| {
        Spi::new_spi1_full_duplex(
            dp.SPI1,
            (gpioa.a5, gpioa.a6, gpioa.a7),
            MODE_1,
            Div64,
            &mut dp.RCC,
            cs,
        )
    });

    // rn8302
    let mut deviceid: [u8; 7] = [0x8F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00];
    rn8302_cs.set_level(PinState::Low);
    unwrap!(rn8302.transfer(&mut deviceid));
    rn8302_cs.set_level(PinState::High);
    defmt::debug!("deviceid ---> {:#04X}", deviceid);
    if deviceid[2] != 0x83 || deviceid[3] != 0x02 || deviceid[4] != 0x00 {
        defmt::warn!("rn8302 no detected");
    }

    let mut workmode: [u8; 4] = [0x81, 0x10, 0x00, 0x00];
    rn8302_cs.set_level(PinState::Low);
    unwrap!(rn8302.transfer(&mut workmode));
    rn8302_cs.set_level(PinState::High);
    defmt::debug!("workmode ---> {:#04X}", workmode);
    defmt::info!("rn8302 {} mode", rn8302_wmstr(workmode[2]));

    loop {
        enable_shutdown_sleeponexit(&mut dp.PWR, &mut cp.SCB);
    }
}
