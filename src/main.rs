#![feature(generic_arg_infer)]
use std::sync::{Mutex, Arc};

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::serial::Uart;
use tfmini_plus::{TFMPError, TFMP};

use crate::tfmini_plus::OutputFormat;

mod tfmini_plus;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let uart = Arc::new(Mutex::new(peripherals.uart1));

    // let sensor_pins = [(pins.gpio18, pins.gpio5)];//, Box::new((pins.gpio32, pins.gpio34))];

    // let mut tfmps = vec![];

    // for (white, green) in sensor_pins {
    //     let uart = unsafe { UART1::new() };
    //     let sensor_serial: serial::Serial<serial::UART1, _, _> = serial::Serial::new(
    //         uart,
    //         serial::Pins {
    //             tx: white,
    //             rx: green,
    //             cts: None,
    //             rts: None,
    //         },
    //         config
    //     ).unwrap();
    //     let tfmp = tfmini_plus::TFMP::new(sensor_serial).unwrap();

    //     tfmps.push(tfmp);
    // }

    // Tx: white, Rx: green
    let mut tfmps = [
        TFMP::new(uart.clone(), pins.gpio25, pins.gpio26).unwrap(),
        // TFMP::new(uart.clone(), pins.gpio32, pins.gpio33).unwrap()
    ];

    for tfmp in &mut tfmps {
        init_tfmp(tfmp).unwrap();
    }

    loop {
        let time = std::time::Instant::now();
        for (i, tfmp) in tfmps.iter_mut().enumerate() {
            print!("TFMP {i}: ");
            let result = tfmp.trigger();
            if let Ok((dist, strength, temp, _)) = result {
                // print!("Data: {dist}, {strength}, {temp}")
            } else {
                print!("Error: {result:?}")
            }
            print!("\t");
        }
        let fps = 1000 / time.elapsed().as_millis();
        println!("\tFPS: {fps}");
    }
}

fn init_tfmp<UART: Uart>(tfmp: &mut tfmini_plus::TFMP<UART>) -> Result<(), TFMPError> {
    let (a, b, c) = tfmp.get_firmware_version()?;
    println!("Firmware: {a}.{b}.{c}");
    tfmp.into_trigger_mode()?;
    tfmp.set_output_format(OutputFormat::MM)?;
    Ok(())
}