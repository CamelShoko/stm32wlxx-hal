// RN8302B energy AFE spi communication

#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::unwrap;
use defmt_rtt as _; // global logger
use hal::spi;
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal,
    cortex_m::{self, delay::Delay},
    embedded_hal::blocking::spi::Transfer,
    embedded_hal::prelude::*,
    gpio::{pins, Output, PinState, PortA},
    pac,
    spi::{BaudRate::Div64, Spi, MODE_1},
    uart::{self, LpUart},
    util::new_delay,
};

struct Rn8302 {
    bus: spi::Spi<pac::SPI1, pins::A5, pins::A6, pins::A7>,
    open: Output<pins::A8>,
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
        ctrl: pins::A8,
    ) -> Self {
        let delay: Delay = new_delay(syst, rcc);

        let chip: Output<pins::A4> = cortex_m::interrupt::free(|cs| (Output::default(ness, cs)));
        let open: Output<pins::A8> = cortex_m::interrupt::free(|cs| (Output::default(ctrl, cs)));

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

    fn delay_ms(&mut self, stable: u32) {
        self.delay.delay_ms(stable);
    }

    fn read_1byte(&mut self, bank: u8, addr: u8) -> u8 {
        let mut input: [u8; 3] = [0; 3];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        // defmt::debug!("reg ---> {:#04X}", input);
        let output = input[2];
        output
    }

    fn read_4byte(&mut self, bank: u8, addr: u8) -> [u8; 4] {
        let mut input: [u8; 6] = [0; 6];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        // defmt::debug!("reg ---> {:#04X}", input);
        let mut output: [u8; 4] = [0; 4];
        for i in 0..4 {
            output[i] = input[2 + i];
        }
        output
    }

    fn read_3byte(&mut self, bank: u8, addr: u8) -> [u8; 3] {
        let mut input: [u8; 5] = [0; 5];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        // defmt::debug!("reg ---> {:#04X}", input);
        let mut output: [u8; 3] = [0; 3];
        for i in 0..3 {
            output[i] = input[2 + i];
        }
        output
    }

    fn write_3byte(&mut self, bank: u8, addr: u8, b0: u8, b1: u8, b2: u8) {
        let mut input: [u8; 6] = [0; 6];
        input[0] = addr;
        input[1] = bank << 4;
        input[2] = b0;
        input[3] = b1;
        input[4] = b2;
        let checksum: u8 = input[0] | input[1] | input[2] | input[3] | input[4];
        input[5] = !checksum;
        defmt::info!("reg <--- {:#04X}", input);
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
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
            let word: u8 = *byte;
            self.bus.write(word).ok();
        }
    }
}

/**
 * [IMPORTANT] probe-rs | probe-run | cargo-embed | cargo-flash
 *
 * cargo build -p examples --target thumbv7em-none-eabi --example rn8302b-3
 * cargo run -p examples --target thumbv7em-none-eabi --example rn8302b-3
 * probe-run --chip STM32WLE5JCIx --connect-under-reset ../../target/thumbv7em-none-eabi/debug/examples/rn8302b-3
 *
 * DEFMT_LOG=trace cargo run-ex rn8302b-3
 *
 * arm-none-eabi-objcopy -v -O binary ../../target/thumbv7em-none-eabi/debug/examples/rn8302b-3 rn8302b-3.bin
 * probe-run --list-probes | probe-run --list-chips
 */
#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);

    let mut led: Output<pins::A1> = cortex_m::interrupt::free(|cs| (Output::default(gpioa.a1, cs)));
    led.set_level(PinState::High);

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
        gpioa.a8,
    );
    rn8302.power_on(50);
    let id = rn8302.read_3byte(0x01, 0x8F);
    defmt::info!("rn8302 id ---> {:#04X}", id);
    let mo = rn8302.read_1byte(0x01, 0x81);
    defmt::info!("rn8302 mo ---> {:#04X}", mo);
    rn8302.write_3byte(0x01, 0x62, 0x77, 0x77, 0x77);

    loop {
        let ai = rn8302.read_4byte(0x00, 0x0B);
        defmt::info!("rn8302 ai ---> {:#04X}", ai);
        let ad = rn8302.read_3byte(0x00, 0x03);
        defmt::info!("rn8302 ad ---> {:#04X}", ad);
        let av = rn8302.read_4byte(0x00, 0x04);
        defmt::info!("rn8302 av ---> {:#04X}", av);
        let ao = rn8302.read_4byte(0x01, 0x24);
        defmt::info!("rn8302 ao ---> {:#04X}", ao);

        if led.level() == PinState::High {
            led.set_level(PinState::Low);
        } else {
            led.set_level(PinState::High);
        }
        rn8302.delay_ms(1000);
    }
}
