#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::_fugit_RateExtU32;
use stm32f4xx_hal::rcc::RccExt;
use stm32f4xx_hal::serial::{Config, Serial, TxISR};
use stm32f4xx_hal::time::U32Ext;

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code


    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let mut dp = pac::Peripherals::take().unwrap();

    let gpioc = dp.GPIOD.split();

    let rcc = dp.RCC.constrain();

    let mut led = gpioc.pd12.into_push_pull_output();
    let mut other_led = gpioc.pd13.into_push_pull_output();

    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let gpioa = dp.GPIOA.split();

    let button = gpioa.pa0.into_pull_down_input();

    // let tx_pin = gpioa.pa2.into_alternate();
    //
    // let rx_pin = gpioa.pa3.into_alternate();
    //
    // let mut serial = Serial::new(
    //     dp.USART2,
    //     (tx_pin, rx_pin),
    //     Config::default()
    //         .baudrate(115200.bps())
    //         .parity_none()
    //         .wordlength_8(),
    //     &clocks,
    // );
    //
    //
    // if let Err(_) = serial {
    //     // jakis panic albo cos
    // }
    //
    // let mut serial = serial.unwrap();
    //
    // let (mut tx, mut rx) = serial.split();

    // new thread for the led


    let mut is_other = false;
    let mut is_other_on = false;
    loop {
        if is_other {
            if is_other_on {
                other_led.set_high();
                is_other_on = false;
            } else {
                other_led.set_low();
                is_other_on = true;
            }
        }
        led.set_high();
        asm::delay(8_000_000);
        led.set_low();
        asm::delay(8_000_000);
        is_other = !is_other;
    }
}