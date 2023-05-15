#![no_std]

use embedded_hal::blocking::i2c;

const REG_ID: u8 = 0x01;
const REG_POWER_CTL1: u8 = 0x02;
const REG_POWER_CTL2: u8 = 0x04;
const REG_CLOCKING_CTL: u8 = 0x05;
const REG_INTERFACE_CTL1: u8 = 0x06;
// const REG_INTERFACE_CTL2: u8 = 0x07;
// const REG_PASSTHR_A_SELECT: u8 = 0x08;
// const REG_PASSTHR_B_SELECT: u8 = 0x09;
const REG_ANALOG_ZC_SR_SETT: u8 = 0x0A;
// const REG_PASSTHR_GANG_CTL: u8 = 0x0C;
// const REG_PLAYBACK_CTL1: u8 = 0x0D;
const REG_MISC_CTL: u8 = 0x0E;
const REG_PLAYBACK_CTL2: u8 = 0x0F;
// const REG_PASSTHR_A_VOL: u8 = 0x14;
// const REG_PASSTHR_B_VOL: u8 = 0x15;
const REG_PCMA_VOL: u8 = 0x1A;
const REG_PCMB_VOL: u8 = 0x1B;
// const REG_BEEP_FREQ_ON_TIME: u8 = 0x1C;
// const REG_BEEP_VOL_OFF_TIME: u8 = 0x1D;
// const REG_BEEP_TONE_CFG: u8 = 0x1E;
const REG_TONE_CTL: u8 = 0x1F;
const REG_MASTER_A_VOL: u8 = 0x20;
const REG_MASTER_B_VOL: u8 = 0x21;
const REG_HEADPHONE_A_VOL: u8 = 0x22;
const REG_HEADPHONE_B_VOL: u8 = 0x23;
const REG_SPEAKER_A_VOL: u8 = 0x24;
const REG_SPEAKER_B_VOL: u8 = 0x25;
// const REG_CH_MIXER_SWAP: u8 = 0x26;
// const REG_LIMIT_CTL1: u8 = 0x27;
// const REG_LIMIT_CTL2: u8 = 0x28;
// const REG_LIMIT_ATTACK_RATE: u8 = 0x29;
// const REG_OVF_CLK_STATUS: u8 = 0x2E;
// const REG_BATT_COMPENSATION: u8 = 0x2F;
// const REG_VP_BATTERY_LEVEL: u8 = 0x30;
// const REG_SPEAKER_STATUS: u8 = 0x31;
// const REG_TEMPMONITOR_CTL: u8 = 0x32;
// const REG_THERMAL_FOLDBACK: u8 = 0x33;
// const REG_CHARGE_PUMP_FREQ: u8 = 0x34;

// const ID: u8 = 0xE0;
const ID_MASK: u8 = 0xF8;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    output_device: OutputDevice,
    volume: u8,
    verify_write: bool,
}

impl Config {
    pub fn new() -> Self {
        Self {
            output_device: OutputDevice::Auto,
            volume: 50,
            verify_write: false,
        }
    }

    pub fn output_device(mut self, output_device: OutputDevice) -> Self {
        self.output_device = output_device;
        self
    }

    pub fn volume(mut self, volume: u8) -> Self {
        self.volume = volume;
        self
    }

    pub fn verify_write(mut self, verify_write: bool) -> Self {
        self.verify_write = verify_write;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct CS43L22<I2C> {
    address: u8,
    bus: I2C,
    stopped: bool,
    config: Config,
}

#[derive(Debug)]
pub enum Error<I2CError> {
    I2C(I2CError),
}

pub type Result<T, I2CError> = core::result::Result<T, Error<I2CError>>;

impl<I2CError> From<I2CError> for Error<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2C(value)
    }
}

impl<I2C, I2CError> CS43L22<I2C>
    where
        I2C: i2c::Write<u8, Error = I2CError>,
        I2C: i2c::WriteRead<u8, Error = I2CError>,
{
    pub fn new(bus: I2C, address: u8, config: Config) -> Result<Self, I2CError> {
        let mut cs43l22 = Self {
            address,
            bus,
            stopped: true,
            config,
        };

        // Keep codec powered OFF
        cs43l22.write_register(REG_POWER_CTL1, 0x01)?;

        // Save output device for mute ON/OFF procedure
        cs43l22.write_register(REG_POWER_CTL2, cs43l22.config.output_device)?;

        // clock config auto detect
        cs43l22.write_register(REG_CLOCKING_CTL, 0x81)?;

        // slave mode and audio standard
        cs43l22.write_register(REG_INTERFACE_CTL1, 0x04)?;

        // set master volume
        cs43l22.set_volume(cs43l22.config.volume)?;

        // if speaker is enabled, set mono mode and volume attenuation level
        if cs43l22.config.output_device != OutputDevice::Headphone {
            // set speaker mono mode
            cs43l22.write_register(REG_PLAYBACK_CTL2, 0x06)?;

            // set speaker attenuation
            cs43l22.write_register(REG_SPEAKER_A_VOL, 0x00)?;
            cs43l22.write_register(REG_SPEAKER_B_VOL, 0x00)?;
        }

        /* Additional configuration for the CODEC. These configurations are done to reduce
        the time needed for the Codec to power off. If these configurations are removed,
        then a long delay should be added between powering off the Codec and switching
        off the I2S peripheral MCLK clock (which is the operating clock for Codec).
        If this delay is not inserted, then the codec will not shut down properly and
        it results in high noise after shut down. */

        /* Disable the analog soft ramp */
        cs43l22.write_register(REG_ANALOG_ZC_SR_SETT, 0x00)?;
        /* Disable the digital soft ramp */
        cs43l22.write_register(REG_MISC_CTL, 0x04)?;
        /* Disable the limiter attack level */
        // cs43l22.write_register(REG_LIMIT_CTL1, 0x00)?;
        /* Adjust Bass and Treble levels */
        cs43l22.write_register(REG_TONE_CTL, 0x0F)?;
        /* Adjust PCM volume level */
        cs43l22.write_register(REG_PCMA_VOL, 0x0A)?;
        cs43l22.write_register(REG_PCMB_VOL, 0x0A)?;

        Ok(cs43l22)
    }

    pub fn release(self) -> I2C {
        let Self { bus, .. } = self;
        bus
    }

    pub fn read_id(&mut self) -> Result<u8, I2CError> {
        let id = self.read_register(REG_ID)?;

        Ok(id & ID_MASK)
    }

    pub fn play(&mut self) -> Result<(), I2CError> {
        if self.stopped {
            // Enable digital soft ramp
            self.write_register(REG_MISC_CTL, 0x06)?;
            /* Enable Output device */
            self.set_mute(false)?;

            /* Power on the Codec */
            self.write_register(REG_POWER_CTL1, 0x9E)?;

            self.stopped = false;
        }

        Ok(())
    }

    pub fn pause(&mut self) -> Result<(), I2CError> {
        /* Pause the audio file playing */
        /* Mute the output first */
        self.set_mute(true)?;
        /* Put the Codec in Power save mode */
        self.write_register(REG_POWER_CTL1, 0x01)?;

        Ok(())
    }

    pub fn resume(&mut self) -> Result<(), I2CError> {
        /* Resumes the audio file playing */
        /* Unmute the output first */
        self.set_mute(false)?;

        // Kill time?
        for _ in 0..0xff {}

        self.write_register(REG_POWER_CTL2, self.config.output_device)?;

        /* Exit the Power save mode */
        self.write_register(REG_POWER_CTL1, 0x9E)?;

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), I2CError> {
        /* Mute the output first */
        self.set_mute(true)?;

        /* Disable the digital soft ramp */
        self.write_register(REG_MISC_CTL, 0x04)?;

        /* Power down the DAC and the speaker (PMDAC and PMSPK bits)*/
        self.write_register(REG_POWER_CTL1, 0x9F)?;

        self.stopped = true;

        Ok(())
    }

    pub fn set_volume(&mut self, volume: u8) -> Result<(), I2CError> {
        // TODO: should this stick with 0-100 or trust people to know 0-255?
        debug_assert!(
            (0..=100).contains(&volume),
            "volume must be between 0 and 100"
        );

        self.config.volume = volume;

        // scale volume range from 0..=100 to 0..=255
        let volume = ((volume as usize) * 255 / 100) as u8;

        if volume > 0xE6 {
            self.write_register(REG_MASTER_A_VOL, volume - 0xE7)?;
            self.write_register(REG_MASTER_B_VOL, volume - 0xE7)?;
        } else {
            self.write_register(REG_MASTER_A_VOL, volume + 0x19)?;
            self.write_register(REG_MASTER_B_VOL, volume + 0x19)?;
        }

        Ok(())
    }

    fn set_mute(&mut self, mute: bool) -> Result<(), I2CError> {
        if mute {
            self.write_register(REG_POWER_CTL2, 0xFF)?;
            self.write_register(REG_HEADPHONE_A_VOL, 0x01)?;
            self.write_register(REG_HEADPHONE_B_VOL, 0x01)?;
        } else {
            self.write_register(REG_HEADPHONE_A_VOL, 0x00)?;
            self.write_register(REG_HEADPHONE_B_VOL, 0x00)?;
            self.write_register(REG_POWER_CTL2, self.config.output_device)?;
        }

        Ok(())
    }

    fn write_register<Data: Into<u8>>(&mut self, register: u8, data: Data) -> Result<(), I2CError> {
        let data = data.into();
        // Write MAP(register) byte and data
        self.bus.write(self.address, &[register, data])?;

        // Check that the value was written to the register
        if self.config.verify_write && self.read_register(register)? != data {
            panic!("error writing register({:#04x}): {:#04x}", register, data);
        }

        Ok(())
    }

    fn read_register(&mut self, register: u8) -> Result<u8, I2CError> {
        // Write MAP byte and then read data
        let mut bytes = [0];
        self.bus.write_read(self.address, &[register], &mut bytes)?;
        Ok(bytes[0])
    }
}

#[cfg(feature = "defmt")]
impl<I2C> defmt::Format for CS43L22<I2C> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "CS43L22 {{ ");
        defmt::write!(fmt, "address: {}, ", self.address);
        defmt::write!(fmt, "stopped: {}, ", self.stopped);
        defmt::write!(fmt, "bus: I2C, ");
        defmt::write!(fmt, "config: {} ", self.config);
        defmt::write!(fmt, "}}");
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDevice {
    Speaker,
    Headphone,
    Both,
    Auto,
}

impl OutputDevice {
    fn value(self) -> u8 {
        match self {
            Self::Speaker => 0xFA,
            Self::Headphone => 0xAF,
            Self::Both => 0xAA,
            Self::Auto => 0x05,
        }
    }
}

impl From<OutputDevice> for u8 {
    fn from(device: OutputDevice) -> Self {
        device.value()
    }
}