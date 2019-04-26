extern crate linux_embedded_hal as hal;
extern crate sht2x;

use hal::{Delay, I2cdev};
use sht2x::SHT2x;

fn main() {
    println!("Hello, SHT2x!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sht21 = SHT2x::new(dev, Delay);

    println!("Temperature raw: {:?}", sht21.temperature().unwrap());
}
