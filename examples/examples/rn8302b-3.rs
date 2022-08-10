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
    gpio::{pins, Output, PinState, PortA, PortB},
    info::Uid,
    pac,
    spi::{BaudRate::Div64, Spi, MODE_1},
    uart::{self, LpUart, Uart1},
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

    fn read_4byte(&mut self, bank: u8, addr: u8) -> [u8; 4] {
        let mut input: [u8; 6] = [0; 6];
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

    fn read_3byte(&mut self, bank: u8, addr: u8) -> [u8; 3] {
        let mut input: [u8; 5] = [0; 5];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        defmt::debug!("reg ---> {:#04X}", input);
        let mut output: [u8; 3] = [0; 3];
        for i in 0..3 {
            output[i] = input[2 + i];
        }
        output
    }

    fn read_2byte(&mut self, bank: u8, addr: u8) -> [u8; 2] {
        let mut input: [u8; 4] = [0; 4];
        input[0] = addr;
        input[1] = bank << 4;
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
        defmt::debug!("reg ---> {:#04X}", input);
        let mut output: [u8; 2] = [0; 2];
        for i in 0..2 {
            output[i] = input[2 + i];
        }
        output
    }

    #[allow(dead_code)]
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

    #[allow(dead_code)]
    fn write_3byte(&mut self, bank: u8, addr: u8, b0: u8, b1: u8, b2: u8) {
        let mut key: [u8; 4] = [0x80, 0x90, 0xE5, 0x0A];
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut key));
        self.chip.set_level(PinState::High);

        let mut input: [u8; 6] = [0; 6];
        input[0] = addr;
        input[1] = 0x80 | (bank << 4);
        input[2] = b0;
        input[3] = b1;
        input[4] = b2;
        let mut sum: u32 = 0;
        for i in 0..5 {
            sum += input[i] as u32;
        }
        input[5] = 0xFF - (sum as u8);
        defmt::debug!("reg <--- {:#04X}", input);
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
    }

    fn write_1byte(&mut self, bank: u8, addr: u8, b0: u8) {
        let mut key: [u8; 4] = [0x80, 0x90, 0xE5, 0x0A];
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut key));
        self.chip.set_level(PinState::High);

        let mut input: [u8; 4] = [0; 4];
        input[0] = addr;
        input[1] = 0x80 | (bank << 4);
        input[2] = b0;
        let mut sum: u32 = 0;
        for i in 0..3 {
            sum += input[i] as u32;
        }
        input[3] = 0xFF - (sum as u8);
        defmt::debug!("reg <--- {:#04X}", input);
        self.chip.set_level(PinState::Low);
        unwrap!(self.bus.transfer(&mut input));
        self.chip.set_level(PinState::High);
    }

    fn i_convert_float(&mut self, hex: &[u8], t: u8) -> f32 {
        let mut i: u32 =
            (hex[0] as u32) << 24 | (hex[1] as u32) << 16 | (hex[2] as u32) << 8 | (hex[3] as u32);
        if i > 7500 {
            i = i - 7500;
        }
        // N: 35855 A: 1835.0
        let mut ratio: f32 = 1.0;
        if t == 1 || t == 2 || t == 3 {
            ratio = 1845.0;
        }
        else
        if t == 4 {
            ratio = 35855.0;
        }
        let c: f32 = (i as f32) / ratio;
        c
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
            LpUart::new(lpuart, 230400, uart::Clk::Hsi16, rcc)
                .enable_rx(rx, cs)
                .enable_tx(tx, cs)
        });
        defmt::info!("serial bus init done");
        Serial { bus }
    }

    #[allow(dead_code)]
    fn send_string(&mut self, s: &str) {
        defmt::debug!("input string {}", s);
        unwrap!(write!(self.bus, "{}", s).ok());
    }

    #[allow(dead_code)]
    fn send_hex(&mut self, hex: &[u8]) {
        defmt::debug!("input hex {:#04X}", hex);
        for byte in hex.into_iter() {
            let word: u8 = *byte;
            self.bus.write(word).ok();
        }
    }
}

struct Line {
    bus: Uart1<pins::B7, pins::B6>,
}

impl Line {
    #[inline]
    pub fn new(uart1: pac::USART1, rcc: &mut pac::RCC, rx: pins::B7, tx: pins::B6) -> Self {
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().is_not_ready() {}
        let bus = cortex_m::interrupt::free(|cs| {
            Uart1::new(uart1, 230400, uart::Clk::Hsi16, rcc)
                .enable_rx(rx, cs)
                .enable_tx(tx, cs)
        });
        defmt::info!("line bus init done");
        Line { bus }
    }

    #[allow(dead_code)]
    fn send_string(&mut self, s: &str) {
        defmt::debug!("input string {}", s);
        unwrap!(write!(self.bus, "{}", s).ok());
    }

    #[allow(dead_code)]
    fn send_hex(&mut self, hex: &[u8]) {
        defmt::debug!("input hex {:#04X}", hex);
        for byte in hex.into_iter() {
            let word: u8 = *byte;
            self.bus.write(word).ok();
        }
    }
}

const PROTOCOL_VER: &str = "V10.01";

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
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);

    let mut led: Output<pins::A1> = cortex_m::interrupt::free(|cs| (Output::default(gpioa.a1, cs)));
    led.set_level(PinState::High);

    // serial
    let mut serial: Serial = Serial::new(dp.LPUART, &mut dp.RCC, gpioa.a3, gpioa.a2);

    // line
    let mut line: Line = Line::new(dp.USART1, &mut dp.RCC, gpiob.b7, gpiob.b6);

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

    rn8302.write_1byte(0x01, 0x81, 0xA2);
    rn8302.write_1byte(0x01, 0x82, 0xFA);
    rn8302.delay_ms(50);

    let status = rn8302.read_2byte(0x01, 0x8A);
    defmt::info!("rn8302 status ---> {:#04X}", status);
    let uid = Uid::from_device().lot();
    let mut seq: u32 = 0;

    loop {
        let nii = rn8302.read_4byte(0x00, 0x0B);
        let nfi = rn8302.i_convert_float(&nii, 4);
        defmt::info!("rn8302 NI ---> {:#04X} {}", nii, nfi);

        let aii = rn8302.read_4byte(0x00, 0x0E);
        let afi = rn8302.i_convert_float(&aii, 1);
        defmt::info!("rn8302 AI ---> {:#04X} {}", aii, afi);

        let bii = rn8302.read_4byte(0x00, 0x0D);
        let bfi = rn8302.i_convert_float(&bii, 2);
        defmt::info!("rn8302 BI ---> {:#04X} {}", bii, bfi);

        let cii = rn8302.read_4byte(0x00, 0x0C);
        let cfi = rn8302.i_convert_float(&cii, 3);
        defmt::info!("rn8302 CI ---> {:#04X} {}", cii, cfi);

        // line
        unwrap!(write!(
            line.bus,
            r#"{{"seq":{},"id":"{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}","ver":"{}","I":{{"A":{},"B":{},"C":{},"N":{},"unit":"mA"}}}}"#,
            (seq as u32),
            uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6],
            PROTOCOL_VER,
            (afi as u32),
            (bfi as u32),
            (cfi as u32),
            (nfi as u32)
        )
        .ok());
        line.send_hex(&[0x0D, 0x0A]);

        // serial
        unwrap!(write!(
            serial.bus,
            r#"{{"seq":{},"id":"{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}","ver":"{}","I":{{"A":{},"B":{},"C":{},"N":{},"unit":"mA"}}}}"#,
            (seq as u32),
            uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6],
            PROTOCOL_VER,
            (afi as u32),
            (bfi as u32),
            (cfi as u32),
            (nfi as u32)
        )
        .ok());
        serial.send_hex(&[0x0D, 0x0A]);
        seq = (seq + 1) as u32;

        led.set_level(PinState::High);
        rn8302.delay_ms(50);
        led.set_level(PinState::Low);
        rn8302.delay_ms(450);
    }
}
