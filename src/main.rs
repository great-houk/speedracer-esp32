#![feature(generic_arg_infer)]
use std::sync::{Arc, Mutex};
use std::time::Instant;

use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use esp_idf_hal::serial::Uart;
use esp_idf_hal::{delay::FreeRtos, prelude::*};
use esp_idf_sys as _; use servo::Servo;
// If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use tfmini_plus::{TFMPError, TFMP};

use crate::tfmini_plus::OutputFormat;

mod tfmini_plus;
mod servo;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // --------------
    // Set up sensors
    // --------------
    
    let uart = Arc::new(Mutex::new(peripherals.uart1));

    // Tx: white, Rx: green
    let mut tfmps = [
        TFMP::new(uart.clone(), pins.gpio18, pins.gpio5).unwrap(), // Left
        TFMP::new(uart.clone(), pins.gpio16, pins.gpio17).unwrap(), // Right
        TFMP::new(uart.clone(), pins.gpio23, pins.gpio22).unwrap(), // Front
    ];

    for tfmp in &mut tfmps {
        init_tfmp(tfmp).unwrap();
    }

    // ---------------
    // Set up steering
    // ---------------

    let mut steering_servo = Servo::new(peripherals.ledc.timer0, peripherals.ledc.channel0, pins.gpio32).unwrap();
    
    // ------------
    // Set up motor
    // ------------
    
    let mut motor = Servo::new(peripherals.ledc.timer1, peripherals.ledc.channel1, pins.gpio26).unwrap();
    

    // Testing
    //---------
    let mut last_l = (0, 0, 0, vec![]);
    let mut last_r = (0, 0, 0, vec![]);
    let mut last_f = (0, 0, 0, vec![]);


    loop {
        // let time = std::time::Instant::now();

        // Go forward slowlyish
        steering_servo.set_us(1500).unwrap();
        motor.set_us(1564).unwrap();

        // print_pretty_output(&mut tfmps);
        [last_l, last_r, last_f] = read_sensors(tfmps).unwrap();


        // So that watchdog doesn't get triggered
        FreeRtos.delay_ms(1u32);
        // let fps = (1_000 * 1_000)
        //     / (match time.elapsed().as_micros() {
        //         0 => 1,
        //         o => o,
        //     });
        // println!("\tFPS: {fps}");
    }
}

fn fps_test(tfmps: &mut [tfmini_plus::TFMP<impl Uart>]) -> () {
    let mut max_fps = 0;
    let mut max_rate = 0;
    const ITERS: u128 = 100;

    for rate in (300..501).step_by(10) {
        for tfmp in tfmps.iter_mut() {
            while let Err(_) = tfmp.set_framerate(rate) {}
        }
        let time = Instant::now();
        for _ in 0..ITERS {
            for tfmp in tfmps.iter_mut() {
                while let Err(_) = tfmp.read() {}
            }
        }
        let fps = (1_000 * 1_000 * ITERS)
            / (match time.elapsed().as_micros() {
                0 => 1,
                o => o,
            });
        if fps > max_fps {
            max_fps = fps;
            max_rate = rate;
        }
        println!("Rate {rate} has fps {fps}");
        FreeRtos.delay_ms(1u32);
    }
    println!("Max FPS ({max_fps}) at rate {max_rate}");
}

fn print_pretty_output(tfmps: &mut [tfmini_plus::TFMP<impl Uart>]) -> () {
    for (i, tfmp) in tfmps.iter_mut().enumerate() {
        print!("TFMP {i}: ");
        let latency = std::time::Instant::now();
        let result = tfmp.read();
        // let result = tfmp.read_pixhawk();
        print!("Latency: {}    ", latency.elapsed().as_micros());
        if let Ok((dist, strength, temp, _)) = result {
            print!("Data: {dist}, {strength}, {temp}")
        // if let Ok(s) = result {
        //     print!("{s}")
        } else {
            print!("Error: {result:?}")
        }
        print!("\t");
    }
}

fn read_sensors<const S>(sensors: &[TFMP; S]) -> [(u16, u16, u16, Vec<u8>); S] {
    let mut rets = [_; S];
    for (i, tfmp) in sensors.iter().enumerate() {
        rets[i] = tfmp.read().unwrap();
    }
    rets
}

fn init_tfmp<UART: Uart>(tfmp: &mut tfmini_plus::TFMP<UART>) -> Result<(), TFMPError> {
    let (a, b, c) = tfmp.get_firmware_version()?;
    println!("Firmware: {a}.{b}.{c}");
    // 500 Works best for some reason, any lower is suboptimal, and any higher ruins it
    tfmp.set_framerate(500)?;
    // tfmp.into_trigger_mode()?;
    tfmp.set_output_format(OutputFormat::CM)?;
    Ok(())
}
