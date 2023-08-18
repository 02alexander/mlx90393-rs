# mlx90393-rs
A driver for the mlx90393 sensor that uses `embedded_hal` has has support for both
`I2C` and `SPI`.
## Examples
# Using SPI
```rs
use defmt::info;
use embedded_hal::{blocking::{spi::Transfer, delay::DelayMs}, digital::v2::OutputPin};
use mlx90393::{Magnetometer, SPIInterface};

pub fn example_spi<E, WR>(
    spi: &mut WR,
    delay: &mut impl DelayMs<u32>,
    cs_pin: impl OutputPin,
) -> Result<(), mlx90393::Error<E>>
where
    WR: Transfer<u8, Error = E>,
{
    let spi_interface = SPIInterface { spi, cs: cs_pin };
    let mut sensor = Magnetometer::default_settings(spi_interface)?;
    sensor.set_resolution(mlx90393::Resolution::RES4)?;
    sensor.set_oversampling_ratio(mlx90393::OverSamplingRatio::OSR8)?;
    sensor.set_filter(mlx90393::DigitalFilter::DF8)?;

    match sensor.do_measurement(delay) {
        Ok((t, x, y, z)) => {
            let angle = libm::atan2f(y as f32, x as f32);
            // Do stuff with the data...
            info!("{} {} {} {}", x, y, z, t);
            info!("angle = {}", angle * 180.0 / core::f32::consts::PI);
        }
        Err(e) => {
            // Handle error.
        }
    }

    // You can also start a measurement and collect it later.
    sensor.start_measurement();

    // Do other stuff

    // If it's not done it will return error and you will try to collect it later.
    let (t, x, y, z) = sensor.collect_measurement()?;

    Ok(())
}

```

