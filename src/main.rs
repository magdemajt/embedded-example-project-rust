#![no_std]
#![no_main]

use core::cell::RefCell;
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger


use core::fmt::Write;
use cortex_m::asm;
use cortex_m::interrupt::Mutex;

use cortex_m_rt::{entry, exception, ExceptionFrame};
use stm32f4xx_hal::gpio::{Edge, ExtiPin, GpioExt, Input};
use stm32f4xx_hal::{gpio, pac};
use pac::interrupt;
use stm32f4xx_hal::prelude::_fugit_RateExtU32;
use stm32f4xx_hal::rcc::RccExt;
use stm32f4xx_hal::serial::{Config, Serial, TxISR};
use stm32f4xx_hal::syscfg::SysCfgExt;
use stm32f4xx_hal::time::U32Ext;
use stm32f4xx_hal::timer::TimerExt;
use cortex_m_semihosting::{hio, hprintln};


type ButtonPin = gpio::PA0<Input>;

static BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static LED_ACTIVATOR: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

static DELAY_MS: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        LED_ACTIVATOR.borrow(cs).replace(true);
        let mut button = BUTTON.borrow(cs).borrow_mut();
        if let Some(button) = button.as_mut() {
            button.clear_interrupt_pending_bit();
        }
    });
}

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
    let mut other_other_led = gpioc.pd14.into_push_pull_output();
    let mut other_other_other_led = gpioc.pd15.into_push_pull_output();

    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let delay = dp.TIM1.delay_ms(&clocks);

    // setting up interrupts

    let mut syscfg = dp.SYSCFG.constrain();




    let gpioa = dp.GPIOA.split();

    let mut button = gpioa.pa0.into_pull_down_input();

    button.make_interrupt_source(&mut syscfg);

    button.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

    button.enable_interrupt(&mut dp.EXTI);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(button.interrupt());
    }

    cortex_m::interrupt::free(|cs| {
        BUTTON.borrow(cs).replace(Some(button));
    });

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
    let mut is_all = false;

    let mut should_switch = false;

    // hprintln!("Hello, world!");

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut activator = LED_ACTIVATOR.borrow(cs).borrow_mut();
            if *activator {
                should_switch = true;
                *activator = false;
            }

        });
        if should_switch {
            should_switch = false;
            if !is_all {
                led.set_high();
                other_led.set_high();
                other_other_led.set_high();
                other_other_other_led.set_high();
                is_all = true;
            } else {
                led.set_low();
                other_led.set_low();
                other_other_led.set_low();
                other_other_other_led.set_low();
                is_all = false;
            }
        }
        if is_all {
            continue;
        }
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
        asm::delay(4_000_000);
        led.set_low();
        asm::delay(4_000_000);
        is_other = !is_other;
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }

    loop {}
}