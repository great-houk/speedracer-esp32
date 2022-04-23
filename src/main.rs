use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::serial;

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
            tx: pins.gpio19, // White
            rx: pins.gpio18, // Green
            cts: None,
            rts: None,
        },
        config
    ).unwrap();

    let mut tfmp = tfmini_plus::TFMP::new(sensor_serial).unwrap();
    let (a, b, c) = tfmp.get_firmware_version().unwrap();
    println!("Firmware: {a}.{b}.{c}");
    println!("Testing Reset...");
    // tfmp.system_reset().unwrap();
    println!("Reset Done!");
    tfmp.change_framerate(0).unwrap();
    FreeRtos.delay_ms(100u32);
    println!("Trigger Reading: {:?}", tfmp.trigger().unwrap());
    FreeRtos.delay_ms(1u32);
    println!("Changing Output to mm");
    tfmp.set_output(tfmini_plus::OutputFormat::MM).unwrap();
    println!("Enabling Output...");
    tfmp.enable_output(true).unwrap();
    FreeRtos.delay_ms(1u32);
    println!("Restoring Factory Settings, because I just destroyed this");
    tfmp.reset_factory_settings().unwrap();
    tfmp.change_framerate(10).unwrap();

    loop {
        let (dist, strength, temp, buf) = tfmp.read().unwrap();
        println!("\r\n\r\nDist: {dist}\r\nStrength: {strength}\r\nTemp: {temp}\r\nBuf: {buf:?}\r\n");
    }
}
