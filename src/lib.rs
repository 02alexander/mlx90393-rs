#![cfg_attr(not(test), no_std)]

/*
    SB = 0x10,
    SW = 0x20,
    SM = 0x30,
    RM = 0x40,
    RR = 0x50,
    WR = 0x60,
    EX = 0x80,
    HR = 0xd0,
    HS = 0xe0,
    RT = 0xf0,

*/

use embedded_hal::blocking::delay::DelayMs;

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
pub enum Error {
    IO,
    Sensor(u8),
}

pub struct Magnetometer<'a, D> {
    address: u8,
    i2c: &'a mut D,
    gain: Gain,
    resolution_x: Resolution,
    resolution_y: Resolution,
    resolution_z: Resolution,
    digital_filter: DigitalFilter,
    oversampling_ratio: OverSamplingRatio,
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

#[derive(Clone, Copy, Debug)]
pub enum Resolution {
    RES1 = 0,
    RES2,
    RES4,
    RES8,
}

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

impl<'a, D> Magnetometer<'a, D>
where
    D: embedded_hal::blocking::i2c::WriteRead,
{
    pub fn new_raw(address: u8, i2c: &'a mut D) -> Magnetometer<'a, D> {
        Magnetometer {
            address,
            i2c,
            gain: Gain::X5,
            resolution_x: Resolution::RES1,
            resolution_y: Resolution::RES1,
            resolution_z: Resolution::RES1,
            digital_filter: DigitalFilter::DF1,
            oversampling_ratio: OverSamplingRatio::OSR1,
        }
    }

    /// Sets the resolution. The ADC gives a 19 bit value and the resolution
    /// determines which part goes into the u16 where Resolution::RES8 results in
    /// the most MSb being included.
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error> {
        self.set_resolution_axis(Axis::X, resolution)?;
        self.set_resolution_axis(Axis::Y, resolution)?;
        self.set_resolution_axis(Axis::Z, resolution)?;
        Ok(())
    }

    /// Sets the resolution. The ADC gives a 19 bit value and the resolution
    /// determines which part goes into the u16 where Resolution::RES8 results in
    /// the most MSb being included.
    pub fn set_resolution_axis(&mut self, axis: Axis, resolution: Resolution) -> Result<(), Error> {
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
        let mask = !((0b11 as u16) << offset);
        let new_reg = (reg & mask) | ((resolution as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        Ok(())
    }

    /// Sets the digital filter. Higher value on filter leads to more accurate values but also takes more time.
    pub fn set_filter(&mut self, filter: DigitalFilter) -> Result<(), Error> {
        let reg = self.memory_read(0x02)?;
        let offset = 2;
        let mask = !((0b1111 as u16) << offset);
        let new_reg = (reg & mask) | ((filter as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        Ok(())
    }

    /// Sets the oversampling ratio. Higher ratio leads to more accurate values but also takes more time.
    pub fn set_magnetic_oversampling_ratio(
        &mut self,
        ratio: OverSamplingRatio,
    ) -> Result<(), Error> {
        let reg = self.memory_read(0x02)?;
        let offset = 0;
        let mask = !((0b11 as u16) << offset);
        let new_reg = (reg & mask) | ((ratio as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        Ok(())
    }

    /// Sets the oversampling ratio. Higher ratio leads to more accurate values but also takes more time.
    pub fn set_temperature_oversampling_ratio(
        &mut self,
        ratio: OverSamplingRatio,
    ) -> Result<(), Error> {
        let reg = self.memory_read(0x02)?;
        let offset = 0;
        let mask = !((0b11 as u16) << offset);
        let new_reg = (reg & mask) | ((ratio as u16) << offset);
        self.memory_write(0x02, new_reg)?;
        Ok(())
    }

    /// Starts a measurement.
    pub fn start_measurement(&mut self) -> Result<(), Error> {
        let mut buffer = [0 as u8; 1];
        let cmd_byte = Command::SM as u8 | 0xf;
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;

        status_err(buffer[0])?;
        Ok(())
    }

    /// Collects the data from a measurement. `start_measurment` must be called before to start one.
    pub fn collect_measurement(&mut self) -> Result<(i16, i16, i16, i16), Error> {
        let mut buffer = [0 as u8; 9];
        let cmd_byte = Command::RM as u8 | 0xf;
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;
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
    ) -> Result<(i16, i16, i16, i16), Error> {
        self.start_measurement().map_err(|_| Error::IO)?;

        delay.delay_ms(300);

        self.collect_measurement()
    }

    /// Stores the current configuration into the non-volatile memory.
    /// NOTE: The non-volatime memory wasn't meant to be written to a lot
    /// so the number of write-cycles should be kept to a minimum.
    pub fn store(&mut self) -> Result<(), Error> {
        let mut buffer = [0 as u8; 1];
        let cmd_byte = Command::HS as u8;
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Copies the configuration from the non-volatile memory to the
    /// volatile memory.
    pub fn recall(&mut self) -> Result<(), Error> {
        let cmd_byte = Command::HR as u8;
        let mut buffer = [0 as u8; 1];
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Cancels the current measurement.
    pub fn exit(&mut self) -> Result<(), Error> {
        let cmd_byte = Command::EX as u8;
        let mut buffer = [0 as u8; 1];
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Resets the device.
    pub fn reset(&mut self) -> Result<(), Error> {
        let cmd_byte = Command::RT as u8;
        let mut buffer = [0 as u8; 1];
        self.i2c
            .write_read(self.address, &[cmd_byte], &mut buffer)
            .map_err(|_| Error::IO)?;
        status_err(buffer[0])?;
        Ok(())
    }

    /// Writes data to the specified address.
    /// Adressses 0x00..=0x09 are used for configuring the device
    /// but 0x0A..=0x1F is free to use.
    pub fn memory_write(&mut self, mem_adress: u8, data: u16) -> Result<(), Error> {
        let cmd_byte = Command::WR as u8;
        let shifted_address = mem_adress << 2;
        let mut buffer = [0 as u8; 1];
        self.i2c
            .write_read(
                self.address,
                &[
                    cmd_byte,
                    (data >> 8) as u8,
                    (data & 0xff) as u8,
                    shifted_address,
                ],
                &mut buffer,
            )
            .map_err(|_| Error::IO)?;
        status_err(buffer[0])?;

        Ok(())
    }

    /// Reads data to the specified address.
    /// Adressses 0x00..=0x09 are used for configuring the device
    /// but 0x0A..=0x1F is free to use.
    pub fn memory_read(&mut self, mem_adress: u8) -> Result<u16, Error> {
        let cmd_byte = Command::RR as u8;
        let shifted_address = mem_adress << 2;
        let mut buffer = [0 as u8; 3];
        self.i2c
            .write_read(self.address, &[cmd_byte, shifted_address], &mut buffer)
            .map_err(|_| Error::IO)?;
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

fn status_err(status: u8) -> Result<(), Error> {
    if status & 16 == 16 {
        Err(Error::Sensor(status))
    } else {
        Ok(())
    }
}

