use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::serial;

use crate::tfmini_plus::OutputFormat;

mod tfmini_plus;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let config = serial::config::Config::default().baudrate(Hertz(115_200));

    let sensor_serial: serial::Serial<serial::UART1, _, _> = serial::Serial::new(
        peripherals.uart1,
        serial::Pins {
            tx: pins.gpio25, // White
            rx: pins.gpio26, // Green
            cts: None,
            rts: None,
        },
        config
    ).unwrap();

    let mut tfmp = tfmini_plus::TFMP::new(sensor_serial).unwrap();

    let (a, b, c) = tfmp.get_firmware_version().unwrap();
    println!("Firmware: {a}.{b}.{c}");
    tfmp.into_trigger_mode().unwrap();
    tfmp.set_output_format(OutputFormat::MM).unwrap();

    loop {
        let result = tfmp.read();
        if let Ok((dist, strength, temp, _)) = result {
            println!("Data: {dist}, {strength}, {temp}")
        } else {
            println!("Error: {result:?}")
        }
    }
}
