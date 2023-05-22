#![no_std]
#![no_main]

pub mod cs43l22;


use core::borrow::Borrow;
use core::cell::RefCell;
use core::cmp::max;
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
use stm32f4xx_hal::{block, gpio, i2s, pac};
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
use stm32f4xx_hal::i2s::stm32_i2s_v12x::transfer::{Data32Channel32, I2sTransfer, I2sTransferConfig, Philips};
use stm32f4xx_hal::interrupt::USART2;
use stm32f4xx_hal::spi::{Phase, Polarity, Spi};
use crate::cs43l22::CS43L22;

const SAMPLE_RATE: u32 = 96_000;

// stm32f411e-disco

struct Angle {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

const ANGLE_THRESHOLD: f32 = 5.0;

impl Angle {
    pub fn new() -> Self {
        Angle {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
    pub fn update(&mut self, x: f32, y: f32, z: f32) {
        if abs(x) > ANGLE_THRESHOLD {
            if self.x + x > 360.0 {
                self.x = self.x + x - 360.0;
            } else {
                self.x += x;
            }
        }
        if abs(y) > ANGLE_THRESHOLD {
            if self.y + y > 360.0 {
                self.y = self.y + y - 360.0;
            } else {
                self.y += y;
            }
        }
        if abs(z) > ANGLE_THRESHOLD {
            if self.z + z > 360.0 {
                self.z = self.z + z - 360.0;
            } else {
                self.z += z;
            }
        }
    }

    pub fn x(&self) -> f32 {
        return self.x;
    }
    pub fn y(&self) -> f32 {
        return self.y;
    }
    pub fn z(&self) -> f32 {
        return self.z;
    }
    pub fn reset(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
        self.z = 0.0;
    }
}

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
    Gyro,
}

impl BoardMode {
    pub fn next_state(&self) -> BoardMode {
        return match self {
            BoardMode::LightingLeds => BoardMode::AllLeds,
            BoardMode::SwitchingOffLeds => BoardMode::Gyro,
            BoardMode::AllLeds => BoardMode::AllLeds,
            BoardMode::Gyro => BoardMode::Gyro,
        };
    }
}

struct Board {
    pub mode: BoardMode,
    pub angle: Angle,
}

impl Board {
    pub fn new() -> Self {
        Board {
            mode: BoardMode::Gyro,
            angle: Angle::new(),
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
        match next_state {
            BoardMode::SwitchingOffLeds => {
                self.angle.reset();
            }
            _ => {}
        };
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

    let gpiod = dp.GPIOD.split();

    let gpiob = dp.GPIOB.split();

    let gpioc = dp.GPIOC.split();

    let gpioe = dp.GPIOE.split();

    let rcc = dp.RCC.constrain();

    // l3 -> orange
    // l5 -> red
    // l4 -> green
    // l6 -> blue

    let mut green_led = gpiod.pd12.into_push_pull_output();
    let mut orange_led = gpiod.pd13.into_push_pull_output();
    let mut red_led = gpiod.pd14.into_push_pull_output();
    let mut blue_led = gpiod.pd15.into_push_pull_output();

    let spi_pins = (
        gpioa.pa5.into_alternate(),  // SCK
        gpioa.pa6.into_alternate(),  // MISO
        gpioa.pa7.into_alternate(),  // MOSI
    );

    let audio_sda = gpiob.pb9;
    let audio_scl = gpiob.pb6;

    let cs43l22_ws = gpioa.pa4;
    let cs43l22_mck = gpioc.pc7;
    let cs43l22_ck = gpioc.pc10;
    let cs43l22_sd = gpioc.pc12;
    let mut cs43l22_reset = gpiod.pd4.into_push_pull_output();

    cs43l22_reset.set_high();

    let mut cs_pin = gpioe.pe3.into_push_pull_output();

    let clocks = rcc.cfgr
        .use_hse(8.MHz())
        .i2s_clk(96.MHz())
        .freeze();

    let i2c1 = dp.I2C1.i2c(
        (audio_scl, audio_sda),
        100.kHz(),
        &clocks,
    );


    // TODO TU JE BUG
    let mut codec = CS43L22::new(i2c1, 0x94, cs43l22::Config::new().volume(100).verify_write(true)).unwrap();
    //
    // codec.play().unwrap();

    let i2s = i2s::I2s::new(dp.SPI3, (
        cs43l22_ws,
        cs43l22_ck,
        cs43l22_mck,
        cs43l22_sd,
    ), &clocks);
    let i2s_config = I2sTransferConfig::new_master()
        .transmit()
        .master_clock(true)
        .standard(Philips)
        .data_format(Data32Channel32)
        .request_frequency(SAMPLE_RATE);


    let mut i2s_transfer = I2sTransfer::new(i2s, i2s_config);


    cs_pin.set_high();

    let spi = Spi::new(
        dp.SPI1,
        spi_pins,
        l3gd20::MODE,
        4.MHz(),
        &clocks,
    );

    let mut gyroscope = L3gd20::new(spi, cs_pin).unwrap();

    gyroscope.set_scale(Scale::Dps500).unwrap();
    gyroscope.set_odr(l3gd20::Odr::Hz760).unwrap();

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

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut activator = LED_ACTIVATOR.borrow(cs).borrow_mut();
            if *activator {
                board.button_click();
                *activator = false;
            }
        });

        match &board.mode {
            BoardMode::AllLeds => {}
            BoardMode::Gyro => {
                let mut angle = &mut board.angle;
                if cycle % 10 == 0 {
                    if angle.x() > 0.5f32 {
                        green_led.set_high();
                        red_led.set_low();
                    } else if angle.x() < -0.5f32 {
                        green_led.set_low();
                        red_led.set_high();
                    }
                    if angle.y() < -0.5f32 {
                        orange_led.set_high();
                        blue_led.set_low();
                    } else if angle.y() > 0.5f32 {
                        orange_led.set_low();
                        blue_led.set_high();
                    }
                }

                if cycle % 50 == 0 {
                    writeln!(tx, "x: {}, y: {}, z: {}", angle.x(), angle.y(), angle.z()).unwrap();
                }

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

                let status_x_float = if status.x_new { 1.0 } else { 0.0 };
                let status_y_float = if status.y_new { 1.0 } else { 0.0 };
                let status_z_float = if status.z_new { 1.0 } else { 0.0 };

                let measurement = gyroscope.all().unwrap();
                let gyro_scale = gyroscope.scale().unwrap();
                let x = gyro_scale.degrees(measurement.gyro.x) * status_x_float;
                let y = gyro_scale.degrees(measurement.gyro.y) * status_y_float;
                let z = gyro_scale.degrees(measurement.gyro.z) * status_z_float;
                angle.update(x, y, z);
            }
            BoardMode::LightingLeds => {
                green_led.set_high();
                orange_led.set_high();
                red_led.set_high();
                blue_led.set_high();
                board.next_state();
            }
            BoardMode::SwitchingOffLeds => {
                green_led.set_low();
                orange_led.set_low();
                red_led.set_low();
                blue_led.set_low();
                board.next_state();
                board.angle.reset();
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