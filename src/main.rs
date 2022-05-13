use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::{serial, serial::{Uart, UART1}};
use esp_idf_hal::gpio::{InputPin, OutputPin};
use tfmini_plus::TFMPError;

use crate::tfmini_plus::OutputFormat;

mod tfmini_plus;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let config = serial::config::Config::default().baudrate(Hertz(115_200));

    let sensor_pins = [(pins.gpio18, pins.gpio5)];

    let mut tfmps = vec![];

    for (white, green) in sensor_pins {
        let uart = unsafe { UART1::new() };
        let sensor_serial: serial::Serial<serial::UART1, _, _> = serial::Serial::new(
            uart,
            serial::Pins {
                tx: white,
                rx: green,
                cts: None,
                rts: None,
            },
            config
        ).unwrap();
        let tfmp = tfmini_plus::TFMP::new(sensor_serial).unwrap();

        tfmps.push(tfmp);
    }

    for tfmp in &mut tfmps {
        init_tfmp(tfmp).unwrap();
    }

    loop {
        let result = tfmps[0].trigger();
        if let Ok((dist, strength, temp, _)) = result {
            println!("Data: {dist}, {strength}, {temp}")
        } else {
            println!("Error: {result:?}")
        }
    }
}

fn init_tfmp<UART: Uart>(tfmp: &mut tfmini_plus::TFMP<UART>) -> Result<(), TFMPError> {
    let (a, b, c) = tfmp.get_firmware_version()?;
    println!("Firmware: {a}.{b}.{c}");
    tfmp.into_trigger_mode()?;
    tfmp.set_output_format(OutputFormat::MM)?;
    Ok(())
}
