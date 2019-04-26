//! Platform agnostic driver for the Sensirion sht2x temperature/humidity sensors.
//!
//! Tested with sht21.
//!
//! This driver is very much inspired by the si7021 driver. Thanks!

#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};


/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Wrong CRC
    Crc,
    /// I2C bus error
    I2c(E),
}

/// I²C address
pub const ADDRESS: u8 = 0x40;

pub struct Temperature(i32);
pub struct Humidity(i32);

impl Temperature {
    pub fn as_millicelsius(&self) -> i32 { self.0 }
    pub fn as_celsius(&self) -> f32 { self.0 as f32 / 1000. }
}

impl Humidity {
    pub fn as_percentmille(&self) -> i32 { self.0 }
    pub fn as_percent(&self) -> f32 { self.0 as f32 / 1000. }
}


/// I²C commands
#[allow(dead_code)]
#[derive(Copy, Clone)]
enum Command {
    MeasureTempHoldMaster = 0xe3,
    MeasureHumiHoldMaster = 0xe5,
    MeasureTempNoHoldMaster = 0xf3,
    MeasureHumiNoHoldMaster = 0xf5,
    WriteUserReg = 0xe6,
    ReadUserReg = 0xe7,
    SoftReset = 0xfe,
}

impl Command {
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

/// Measurement resolution
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    /// RH: 12 bit, Temp: 14 bit
    RH12Temp14 = 0x00,
    /// RH: 8 bit, Temp: 12 bit
    RH8Temp12 = 0x01,
    /// RH: 10 bit, Temp: 13 bit
    RH10Temp13 = 0x10,
    /// RH: 11 bit, Temp: 11 bit
    RH11Temp11 = 0x11,
}

impl Resolution {
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

/// SHT2x Driver
pub struct SHT2x<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> SHT2x<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Initialize the SHT2x driver.
    pub fn new(i2c: I2C, delay: D) -> Self {
        SHT2x { i2c, delay }
    }

    /// Starts a temperature measurement and waits for it to finish before
    /// returning the measured value.
    pub fn temperature(&mut self) -> Result<Temperature, Error<E>> {
        self.command(Command::MeasureTempNoHoldMaster)?;

        // Wait for conversion to finish.
        // Max time for conversion in 14 bit mode according to datasheet
        // is 85ms. TODO: Do this depending on resolution, or do polling.
        self.delay.delay_ms(85);

        let temp_raw = self.read_u16()?; // TODO CRC

        Ok(convert_raw_temperature(temp_raw))
    }

    /// Starts a relative humidity measurement and waits for it to finish before
    /// returning the measured value.
    pub fn rel_humidity(&mut self) -> Result<Humidity, Error<E>> {
        self.command(Command::MeasureHumiNoHoldMaster)?;

        // Wait for conversion to finish.
        // Max time for conversion in 14 bit mode according to datasheet
        // is 85ms. TODO: Do this depending on resolution, or poll.
        self.delay.delay_ms(85); // FIXME check

        let hum_raw = self.read_u16()?; // TODO CRC

        Ok(convert_raw_rh(hum_raw))
    }

    /// Send a command to the device.
    fn command(&mut self, command: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(ADDRESS, &[command.val()])
            .map_err(Error::I2c)
    }

    /// Read an u16 from the device without checking the CRC.
    fn read_u16(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0, 0];
        self.i2c.read(ADDRESS, &mut buffer).map_err(Error::I2c)?;
        Ok(((buffer[0] as u16) << 8) + (buffer[1] as u16))
    }
}

/// Convert raw temperature measurement to milli-degrees celsius.
///
/// Formula (datasheet 6.2): -46.85 + 175.72 * (val / 2^16)
/// Optimized for integer fixed point (3 digits) arithmetic.
fn convert_raw_temperature(temp_raw: u16) -> Temperature {
    Temperature((((((temp_raw & 0xfffc) as u32) * 21965) >> 13) - 46850) as i32)
}

/// Convert raw humidity measurement to thousands of a % of RH.
///
/// Formula (datasheet 6.1): -6 + 125 * (val / 2^16)
/// The implementation is equivalent for fixed point (3 decimal places).
fn convert_raw_rh(humidity_raw: u16) -> Humidity {
    Humidity((((((humidity_raw & 0xfffc) as u32) * 15625) >> 13) - 6000) as i32)
}
