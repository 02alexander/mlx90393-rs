#![no_std]
//! A simple crate for communicating with the magnetic sensor mlx90393.
//! 
//! To communicate with the sensor `Magnetometer` needs an interface that implements
//! `Tranceive`. This is already implemented for `SPIInterface` and `I2CInterface`.
//! Examples of using them are given below:
//! # Examples 
//! ## Using I2C
//! ```no_run
//! use embedded_hal::blocking::{i2c::WriteRead, delay::DelayMs};
//! use mlx90393::{I2CInterface, Magnetometer};
//! 
//! pub fn example_i2c<E, WR>(i2c: &mut WR, delay: &mut impl DelayMs<u32>) -> Result<(), mlx90393::Error<E>>
//! where
//!     WR: WriteRead<Error = E>,
//! {
//!     let i2c_interface = I2CInterface { i2c, address: 0x18 };
//!     let mut sensor = Magnetometer::default_settings(i2c_interface)?;
//!     sensor.set_resolution(mlx90393::Resolution::RES4)?;
//!     sensor.set_oversampling_ratio(mlx90393::OverSamplingRatio::OSR8)?;
//!     sensor.set_filter(mlx90393::DigitalFilter::DF8)?;
//! 
//!     match sensor.do_measurement(delay) {
//!         Ok((t, x, y, z)) => {
//!             // Do stuff with the data...
//!         }
//!         Err(e) => {
//!             // Handle error.
//!         }
//!     }
//! 
//!     // You can also start a measurement and collect it later.
//!     sensor.start_measurement();
//! 
//!     // Do other stuff
//! 
//!     // If it's not done it will return error and you will try to collect it later.
//!     let (t, x, y, z) = sensor.collect_measurement()?;
//! 
//!     Ok(())
//! }
//! ```
//! ## Using SPI
//! ```no_run
//! use defmt::info;
//! use embedded_hal::{blocking::{spi::Transfer, delay::DelayMs}, digital::v2::OutputPin};
//! use mlx90393::{Magnetometer, SPIInterface};
//! 
//! pub fn example_spi<E, WR>(
//!     spi: &mut WR,
//!     delay: &mut impl DelayMs<u32>,
//!     cs_pin: impl OutputPin,
//! ) -> Result<(), mlx90393::Error<E>>
//! where
//!     WR: Transfer<u8, Error = E>,
//! {
//!     let spi_interface = SPIInterface { spi, cs: cs_pin };
//!     let mut sensor = Magnetometer::default_settings(spi_interface)?;
//!     sensor.set_resolution(mlx90393::Resolution::RES4)?;
//!     sensor.set_oversampling_ratio(mlx90393::OverSamplingRatio::OSR8)?;
//!     sensor.set_filter(mlx90393::DigitalFilter::DF8)?;
//! 
//!     match sensor.do_measurement(delay) {
//!         Ok((t, x, y, z)) => {
//!             // Do stuff with the data...
//!         }
//!         Err(e) => {
//!             // Handle error.
//!         }
//!     }
//! 
//!     // You can also start a measurement and collect it later.
//!     sensor.start_measurement();
//! 
//!     // Do other stuff
//! 
//!     // If it's not done it will return error and you will try to collect it later.
//!     let (t, x, y, z) = sensor.collect_measurement()?;
//! 
//!     Ok(())
//! }
//! ```

use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Command {
    SB = 0x10,
    SW = 0x20,
    SM = 0x30, // Do a single measurement.
    RM = 0x40, // Collect a single measurement from the sensor.
    RR = 0x50,
    WR = 0x60,
    EX = 0x80,
    HR = 0xd0, // Copies the content of the non-volatile memory into the volatile memory.
    HS = 0xe0, // Copies the content of the volatile memory into the non-volative memory.
    RT = 0xf0,
}

#[derive(Clone, Copy, Debug)]
pub enum Error<E> {
    CSPin,
    TranceiveIO(E),
    Sensor(u8),
}

/// Available configurations of the gain.
/// The name of the variants only describe the approximate gain, the exact
/// gain depends on the oversampling ratio, resolution and axis which can be
/// extracted from `GAIN_SEL` in the datasheet.
#[derive(Clone, Copy, Debug)]
pub enum Gain {
    X5 = 0,
    X4,
    X3,
    X2_5,
    X2,
    X1_66,
    X1_33,
    X1,
}

/// The ADC gives a 19 bit value but the sensor gives a 16 bit value 
/// and the resolution determines which part gets included in the 16 bit value 
/// where Resolution::RES8 results in the most MSb being included.
#[derive(Clone, Copy, Debug)]
pub enum Resolution {
    RES1 = 0,
    RES2,
    RES4,
    RES8,
}

/// All the available filters. Using higher filters leads to less noise but takes
/// exponentially more time to compute. The exact compute time is in `TCONV`.
#[derive(Clone, Copy, Debug)]
pub enum DigitalFilter {
    DF1 = 0,
    DF2,
    DF3,
    DF4,
    DF5,
    DF6,
    DF7,
    DF8,
}

/// Higher oversampling leads to less noise but takes more time to comupte.
#[derive(Clone, Copy, Debug)]
pub enum OverSamplingRatio {
    OSR1 = 0,
    OSR2 = 1,
    OSR4 = 2,
    OSR8 = 3,
}

#[derive(Clone, Copy, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// `TCONV[DigitalFiltere][OversamplingRatio] = ` the convertion time in ms for a single axis.
pub const TCONV: [[f32; 4]; 8] = [
    [1.27, 1.84, 3.00, 5.30],
    [1.46, 2.23, 3.76, 6.84],
    [1.84, 3.00, 5.30, 9.91],
    [2.61, 4.53, 8.37, 16.05],
    [4.15, 7.60, 14.52, 28.34],
    [7.22, 13.75, 26.80, 52.92],
    [13.36, 26.04, 51.38, 102.07],
    [25.65, 50.61, 100.53, 200.37],
];

/// Trait required for communication with the sensor.
pub trait Tranceive {
    type Error;
    fn tranceive(&mut self, send: &[u8], recv: &mut [u8]) -> Result<(), Error<Self::Error>>;
}

pub struct I2CInterface<'a, D> {
    pub i2c: &'a mut D,
    pub address: u8,
}
pub struct SPIInterface<'a, D, C> {
    pub spi: &'a mut D,
    pub cs: C,
}

pub struct Magnetometer<P> {
    interface: P,
    gain: Gain,
    resolution_x: Resolution,
    resolution_y: Resolution,
    resolution_z: Resolution,
    digital_filter: DigitalFilter,
    oversampling_ratio: OverSamplingRatio,
}

impl<'a, D, E> Tranceive for I2CInterface<'a, D>
where
    D: embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    type Error = E;
    fn tranceive(&mut self, send: &[u8], recv: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.address, send, recv)
            .map_err(|e| Error::TranceiveIO(e))
    }
}

impl<'a, D, C, E> Tranceive for SPIInterface<'a, D, C>
where
    D: embedded_hal::blocking::spi::Transfer<u8, Error = E>,
    C: OutputPin,
{
    type Error = E;
    fn tranceive(&mut self, send: &[u8], recv: &mut [u8]) -> Result<(), Error<E>> {
        self.cs.set_low().map_err(|_| Error::CSPin)?;
        let mut buf = [0; 16]; // The same buffer is used for sending and transmitting data.
        buf[..send.len()].copy_from_slice(send);
        let received = self
            .spi
            .transfer(&mut buf[..(send.len() + recv.len())])
            .map_err(|e| Error::TranceiveIO(e))?;
        recv.copy_from_slice(&received[send.len()..]);
        self.cs.set_high().map_err(|_| Error::CSPin)?;
        Ok(())
    }
}

impl<P, E> Magnetometer<P>
where
    P: Tranceive<Error = E>,
{
    /// This function assumes the memory all is zero except for the `HALLCONF` parameter.
    /// This is an invalid state and the settings should be changed before use.  
    pub fn new_raw(protocol: P) -> Magnetometer<P> {
        Magnetometer {
            interface: protocol,
            gain: Gain::X5,
            resolution_x: Resolution::RES1,
            resolution_y: Resolution::RES1,
            resolution_z: Resolution::RES1,
            digital_filter: DigitalFilter::DF1,
            oversampling_ratio: OverSamplingRatio::OSR1,
        }
    }

    /// New magnetometer driver with some valid and reasonable settings.
    pub fn default_settings(
        protocol: P,
    ) -> Result<Magnetometer<P>, Error<E>> {
        let mut s = Self::new_raw(protocol);
        s.set_gain(Gain::X5)?;
        s.set_resolution(Resolution::RES4)?;
        s.set_filter(DigitalFilter::DF3)?;
        s.set_oversampling_ratio(OverSamplingRatio::OSR2)?;
        Ok(s)
    }

    /// Sets the resolution. The ADC gives a 19 bit value and the resolution
    /// determines which part goes into the u16 where Resolution::RES8 results in
    /// the most MSb being included.
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<E>> {
        self.set_resolution_axis(Axis::X, resolution)?;
        self.set_resolution_axis(Axis::Y, resolution)?;
        self.set_resolution_axis(Axis::Z, resolution)?;
        Ok(())
    }

    /// Sets the resolution. The ADC gives a 19 bit value and the resolution
    /// determines which part goes into the u16 where Resolution::RES8 results in
    /// the most MSb being included.
    pub fn set_resolution_axis(
        &mut self,
        axis: Axis,
        resolution: Resolution,
    ) -> Result<(), Error<E>> {
        let reg = self.memory_read(0x02)?;
        match axis {
            Axis::X => self.resolution_x = resolution,
            Axis::Y => self.resolution_y = resolution,
            Axis::Z => self.resolution_z = resolution,
        }
        let offset = match axis {
            Axis::X => 5,
            Axis::Y => 7,
            Axis::Z => 9,
        };
        let mask = !((0b11_u16) << offset);
        let new_reg = (reg & mask) | ((resolution as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        Ok(())
    }

    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error<E>> {
        let reg = self.memory_read(0x02)?;
        let offset = 4;
        let mask = !((0b111_u16) << offset);
        let new_reg = (reg & mask) | ((gain as u16) << offset);
        self.memory_write(0x00, new_reg)?;
        self.gain = gain;
        Ok(())
    }

    /// Sets the digital filter. Higher value on filter leads to more accurate values but also takes more time.
    pub fn set_filter(&mut self, filter: DigitalFilter) -> Result<(), Error<E>> {
        let reg = self.memory_read(0x02)?;
        let offset = 2;
        let mask = !((0b1111_u16) << offset);
        let new_reg = (reg & mask) | ((filter as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        self.digital_filter = filter;
        Ok(())
    }

    /// Sets the oversampling ratio. Higher ratio leads to more accurate values but also takes more time.
    pub fn set_oversampling_ratio(&mut self, ratio: OverSamplingRatio) -> Result<(), Error<E>> {
        let reg = self.memory_read(0x02)?;
        let offset = 0;
        let mask = !((0b11_u16) << offset);
        let new_reg = (reg & mask) | ((ratio as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        self.oversampling_ratio = ratio;
        Ok(())
    }

    /// Starts a measurement.
    pub fn start_measurement(&mut self) -> Result<(), Error<E>> {
        let mut buffer = [0; 1];
        let cmd_byte = Command::SM as u8 | 0xf;
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;

        status_err(buffer[0])?;
        Ok(())
    }

    /// Collects the data from a measurement. `start_measurment` must be called before to start one.
    pub fn collect_measurement(&mut self) -> Result<(i16, i16, i16, i16), Error<E>> {
        let mut buffer = [0; 9];
        let cmd_byte = Command::RM as u8 | 0xf;
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;
        status_err(buffer[0])?;
        let t = i16::from_be_bytes([buffer[1], buffer[2]]);
        let raw_x = i16::from_be_bytes([buffer[3], buffer[4]]);
        let raw_y = i16::from_be_bytes([buffer[5], buffer[6]]);
        let raw_z = i16::from_be_bytes([buffer[7], buffer[8]]);
        let x = res_offset(raw_x, self.resolution_x);
        let y = res_offset(raw_y, self.resolution_y);
        let z = res_offset(raw_z, self.resolution_z);
        Ok((t, x, y, z))
    }

    /// Starts and completes a measurement.
    /// Returns (t, x, y, z)
    pub fn do_measurement(
        &mut self,
        delay: &mut impl DelayMs<u32>,
    ) -> Result<(i16, i16, i16, i16), Error<E>> {
        self.start_measurement()?;

        delay.delay_ms(
            (TCONV[self.digital_filter as usize][self.oversampling_ratio as usize] * 1.03) as u32
                + 2,
        );

        self.collect_measurement()
    }

    /// Stores the current configuration into the non-volatile memory.
    /// NOTE: The non-volatime memory wasn't meant to be written to a lot
    /// so the number of write-cycles should be kept to a minimum.
    pub fn store(&mut self) -> Result<(), Error<E>> {
        let mut buffer = [0; 1];
        let cmd_byte = Command::HS as u8;
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Copies the configuration from the non-volatile memory to the
    /// volatile memory.
    pub fn recall(&mut self) -> Result<(), Error<E>> {
        let cmd_byte = Command::HR as u8;
        let mut buffer = [0; 1];
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Cancels the current measurement.
    pub fn exit(&mut self) -> Result<(), Error<E>> {
        let cmd_byte = Command::EX as u8;
        let mut buffer = [0; 1];
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Resets the device.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        let cmd_byte = Command::RT as u8;
        let mut buffer = [0; 1];
        self.interface.tranceive(&[cmd_byte], &mut buffer)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Writes data to the specified address.
    /// Adressses 0x00..=0x09 are used for configuring the device
    /// but 0x0A..=0x1F is free to use.
    pub fn memory_write(&mut self, mem_adress: u8, data: u16) -> Result<(), Error<E>> {
        let cmd_byte = Command::WR as u8;
        let shifted_address = mem_adress << 2;
        let mut buffer = [0; 1];
        self.interface.tranceive(
            &[
                cmd_byte,
                (data >> 8) as u8,
                (data & 0xff) as u8,
                shifted_address,
            ],
            &mut buffer,
        )?;
        status_err(buffer[0])?;

        Ok(())
    }

    /// Reads data to the specified address.
    /// Adressses 0x00..=0x09 are used for configuring the device
    /// but 0x0A..=0x1F is free to use.
    pub fn memory_read(&mut self, mem_adress: u8) -> Result<u16, Error<E>> {
        let cmd_byte = Command::RR as u8;
        let shifted_address = mem_adress << 2;
        let mut buffer = [0; 3];
        self.interface
            .tranceive(&[cmd_byte, shifted_address], &mut buffer)?;
        status_err(buffer[0])?;
        Ok(u16::from_be_bytes([buffer[1], buffer[2]]))
    }
}

fn res_offset(raw: i16, resolution: Resolution) -> i16 {
    // No idea why this is needed.
    match resolution {
        Resolution::RES8 => (raw as i32 - 0x4000) as i16,
        Resolution::RES4 => (raw as i32 - 0x8000) as i16,
        _ => raw,
    }
}

fn status_err<E>(status: u8) -> Result<(), Error<E>> {
    if status & 16 == 16 {
        Err(Error::Sensor(status))
    } else {
        Ok(())
    }
}
