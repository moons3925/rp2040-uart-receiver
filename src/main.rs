#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::serial::{Read, Write};
use fugit::RateExtU32;
use hal::gpio::bank0::{Gpio0, Gpio1};
use hal::pac;
use hal::pac::interrupt;
use hal::uart::{DataBits, StopBits, UartConfig};
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::Clock;
use rp_pico::entry;
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::FunctionUart, hal::gpio::PullNone>,
    hal::gpio::Pin<Gpio1, hal::gpio::FunctionUart, hal::gpio::PullNone>,
);

const BUFF_SIZE: usize = 2048;
struct RingBuffer {
    buffer: [u8; BUFF_SIZE],
    read_ptr: usize,
    write_ptr: usize,
}

impl RingBuffer {
    fn read(&mut self) -> u8 {
        let mut byte = 0;
        critical_section::with(|_| {
            byte = self.buffer[self.read_ptr];
            self.read_ptr = (self.read_ptr + 1) & (BUFF_SIZE - 1);
        });
        byte
    }
    fn write(&mut self, c: u8) {
        self.buffer[self.write_ptr] = c;
        self.write_ptr = (self.write_ptr + 1) & (BUFF_SIZE - 1);
    }
    fn readable(&mut self) -> bool {
        let mut b = false;
        critical_section::with(|_| {
            if self.read_ptr == self.write_ptr {
                b = false;
            } else {
                b = true;
            }
        });
        b
    }
    fn writable(&mut self) -> bool {
        if (self.write_ptr + 1) & (BUFF_SIZE - 1) == self.read_ptr {
            return false;
        } else {
            return true;
        }
    }
}

static mut RING: RingBuffer = RingBuffer {
    buffer: [0; BUFF_SIZE],
    read_ptr: 0,
    write_ptr: 0,
};

static mut UART_RECEIVER: Option<hal::uart::Reader<pac::UART0, UartPins>> = None;

fn on_idle(writer: &mut hal::uart::Writer<pac::UART0, UartPins>) {
    unsafe {
        while RING.readable() {
            let c = RING.read();
            if b'a' <= c && c <= b'z' {
                let _ = nb::block!(writer.write(c));
            } else if b'A' <= c && c <= b'Z' {
                let _ = nb::block!(writer.write(c));
            } else if c == 0x0a || c == 0x0d {
                let _ = nb::block!(writer.write(c));
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.reconfigure(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.reconfigure(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let (mut uart_rx, mut uart_tx) = uart.split();

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ);
    }

    uart_rx.enable_rx_interrupt();

    critical_section::with(|_| unsafe {
        UART_RECEIVER = Some(uart_rx);
    });

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        on_idle(&mut uart_tx);
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
    }
}

#[interrupt]
fn UART0_IRQ() {
    unsafe {
        if let Some(ref mut uart_recv) = UART_RECEIVER.as_mut() {
            if let Ok(c) = uart_recv.read() {
                if RING.writable() {
                    RING.write(c);
                }
            }
        }
    }
}
