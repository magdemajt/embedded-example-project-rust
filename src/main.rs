#![no_std]
#![no_main]


use core::borrow::Borrow;
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
use stm32f4xx_hal::gpio::{Edge, ExtiPin, GpioExt, Input, AF2, PinState, AF5, Alternate};
use stm32f4xx_hal::{block, gpio, pac};
use pac::interrupt;
use stm32f4xx_hal::prelude::_fugit_RateExtU32;
use stm32f4xx_hal::rcc::RccExt;
use stm32f4xx_hal::serial::{Config, Serial, SerialExt, TxISR};
use stm32f4xx_hal::syscfg::SysCfgExt;
use stm32f4xx_hal::time::U32Ext;
use stm32f4xx_hal::timer::TimerExt;
use cortex_m_semihosting::{hio, hprintln};
use l3gd20::{L3gd20, Scale};
use stm32f4xx_hal::i2c::{I2cExt, Mode};
use stm32f4xx_hal::interrupt::USART2;
use stm32f4xx_hal::spi::{Phase, Polarity, Spi};


// stm32f411e-disco

fn abs(x: f32) -> f32 {
    if x < 0.0 {
        return -x;
    }
    return x;
}

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

    // l3 -> orange
    // l5 -> red
    // l4 -> green
    // l6 -> blue

    let mut green_led = gpioc.pd12.into_push_pull_output();
    let mut orange_led = gpioc.pd13.into_push_pull_output();
    let mut red_led = gpioc.pd14.into_push_pull_output();
    let mut blue_led = gpioc.pd15.into_push_pull_output();

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

    // gyroscope.set_scale(Scale::Dps2000).unwrap();
    // gyroscope.set_odr(l3gd20::Odr::Hz380).unwrap();

    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();

    let serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().wordlength_8().parity_none().baudrate(115_200.bps()),
        &clocks,
    )
        .unwrap();

    let (mut tx, mut rx) = serial.split();


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

    // hprintln!("Hello, world!");

    writeln!(tx, "Setup successful").unwrap();

    let mut cycle = 0;

    let (mut old_x, mut old_y, mut old_z) = (0f32, 0f32, 0f32);

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

                if cycle % 5 == 0 {
                    let status = gyroscope.status();
                    if let Err(_) = status {
                        writeln!(tx, "Error reading status").unwrap();
                        continue;
                    }
                    let status = status.unwrap();
                    // print new
                    if !status.x_new && !status.y_new && !status.z_new {
                        continue;
                    }
                    let measurement = gyroscope.all().unwrap();
                    let gyro_scale = gyroscope.scale().unwrap();
                    let x = gyro_scale.radians(measurement.gyro.x);
                    let y = gyro_scale.radians(measurement.gyro.y);
                    let z = gyro_scale.radians(measurement.gyro.z);
                    let diff = abs(x - old_x) + abs(y - old_y) +  abs(z - old_z);
                    if diff < 0.015f32 {
                        continue;
                    }

                    // writeln!(tx, "x: {}, y: {}, z: {}", &x, &y, &z).unwrap();
                    // writeln!(tx, "diff: x: {}, y: {}, z: {}", x - old_x, y - old_y, z - old_z).unwrap();

                    if abs(x - old_x) > 0.01f32 {
                        if x - old_x > 0f32 {
                            red_led.set_low();
                            green_led.set_high();
                        } else if x - old_x < 0f32 {
                            green_led.set_low();
                            red_led.set_high();
                        }
                    }

                    if abs(y - old_y) > 0.01f32 {
                        if y - old_y > 0f32 {
                            blue_led.set_low();
                            orange_led.set_high();
                        } else if y - old_y < 0f32 {
                            orange_led.set_low();
                            blue_led.set_high();
                        }
                    }

                    old_x = x;
                    old_y = y;
                    old_z = z;

                }

            },
            BoardMode::LightingLeds => {
                green_led.set_high();
                orange_led.set_high();
                red_led.set_high();
                blue_led.set_high();
                board.next_state();
            },
            BoardMode::SwitchingOffLeds => {
                green_led.set_low();
                orange_led.set_low();
                red_led.set_low();
                blue_led.set_low();
                board.next_state();
                asm::delay(1_000_000);
            }
        };


        cycle += 1;
        cycle %= 1600;
        asm::delay(10);
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }

    loop {}
}