#![no_std]
#![no_main]


use core::cell::RefCell;
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger


use core::fmt::Write;
use cortex_m::{asm, iprintln};
use cortex_m::interrupt::Mutex;

use cortex_m_rt::{entry, exception, ExceptionFrame};
use stm32f4xx_hal::gpio::{Edge, ExtiPin, GpioExt, Input, AF2, PinState};
use stm32f4xx_hal::{gpio, pac};
use pac::interrupt;
use stm32f4xx_hal::prelude::_fugit_RateExtU32;
use stm32f4xx_hal::rcc::RccExt;
use stm32f4xx_hal::serial::{Config, Serial, TxISR};
use stm32f4xx_hal::syscfg::SysCfgExt;
use stm32f4xx_hal::time::U32Ext;
use stm32f4xx_hal::timer::TimerExt;
use cortex_m_semihosting::{hio, hprintln};
use l3gd20::L3gd20;
use stm32f4xx_hal::i2c::{I2cExt, Mode};
use stm32f4xx_hal::spi::{Phase, Polarity, Spi};


// stm32f411e-disco

enum BoardMode {
    LightingLeds,
    SwitchingOffLeds,
    AllLeds,
    Gyro
}

impl BoardMode {
    pub fn next_state(&self) -> BoardMode {
        return match self {
            BoardMode::LightingLeds => BoardMode::AllLeds,
            BoardMode::SwitchingOffLeds => BoardMode::Gyro,
            BoardMode::AllLeds => BoardMode::AllLeds,
            BoardMode::Gyro => BoardMode::Gyro,
        }
    }
}

struct Board {
    pub mode: BoardMode,
}

impl Board {
    pub fn new() -> Self {
        Board {
            mode: BoardMode::Gyro,
        }
    }

    pub fn set_mode(&mut self, mode: BoardMode) {
        self.mode = mode;
    }

    pub fn button_click(&mut self) {
        match self.mode {
            BoardMode::LightingLeds => self.mode = BoardMode::SwitchingOffLeds,
            BoardMode::SwitchingOffLeds => self.mode = BoardMode::LightingLeds,
            BoardMode::AllLeds => self.mode = BoardMode::SwitchingOffLeds,
            BoardMode::Gyro => self.mode = BoardMode::LightingLeds,
        }
    }

    pub fn get_mode(&self) -> &BoardMode {
        &self.mode
    }

    pub fn next_state(&mut self) {
        let next_state = self.mode.next_state();
        self.mode = next_state;
    }
}

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

    let mut board = Board::new();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let mut dp = pac::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();

    let gpioc = dp.GPIOD.split();

    let gpiob = dp.GPIOB.split();

    let gpioe = dp.GPIOE.split();

    let rcc = dp.RCC.constrain();

    let mut led = gpioc.pd12.into_push_pull_output();
    let mut other_led = gpioc.pd13.into_push_pull_output();
    let mut other_other_led = gpioc.pd14.into_push_pull_output();
    let mut other_other_other_led = gpioc.pd15.into_push_pull_output();

    let spi_pins = (
        gpioa.pa5.into_alternate(),  // SCK
        gpioa.pa6.into_alternate(),  // MISO
        gpioa.pa7.into_alternate(),  // MOSI
    );

    let mut cs_pin = gpioe.pe3.into_push_pull_output();

    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    cs_pin.set_high();

    let spi = Spi::new(
        dp.SPI1,
        spi_pins,
        l3gd20::MODE,
        1.MHz(),
        &clocks,
    );
    let mut gyroscope = L3gd20::new(spi, cs_pin).unwrap();

    let stim = &mut cp.ITM.stim[0];



    // setup gyroscope



    let delay = dp.TIM1.delay_ms(&clocks);

    // setting up interrupts

    let mut syscfg = dp.SYSCFG.constrain();




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


    let mut is_other = false;
    let mut is_other_on = false;

    // hprintln!("Hello, world!");

    let mut cycle = 0;

    loop {

        cortex_m::interrupt::free(|cs| {
            let mut activator = LED_ACTIVATOR.borrow(cs).borrow_mut();
            if *activator {
                board.button_click();
                *activator = false;
            }
        });

        match &board.mode {
            BoardMode::AllLeds => {

            },
            BoardMode::Gyro => {

                let status = gyroscope.status().unwrap();

                if status.new_data {
                    let measurement = gyroscope.all().unwrap();
                    other_led.set_high();
                }

            },
            BoardMode::LightingLeds => {
                led.set_high();
                other_led.set_high();
                other_other_led.set_high();
                other_other_other_led.set_high();
                board.mode.next_state();
            },
            BoardMode::SwitchingOffLeds => {
                led.set_low();
                other_led.set_low();
                other_other_led.set_low();
                other_other_other_led.set_low();
                board.mode.next_state();
            }
        };


        cycle += 1;
        cycle %= 16_000;
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }

    loop {}
}