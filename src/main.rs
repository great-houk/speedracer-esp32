use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::serial;
use std::time::Duration;
use embedded_hal::serial::Read;
use std::thread;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let config = serial::config::Config::default().baudrate(Hertz(115_200));

    let mut sensor_serial: serial::Serial<serial::UART1, _, _> = serial::Serial::new(
        peripherals.uart1,
        serial::Pins {
            tx: pins.gpio19, // White
            rx: pins.gpio18, // Green
            cts: None,
            rts: None,
        },
        config
    ).unwrap();
    // let (mut tx, mut rx) = sensor_serial.split();

    let mut tfmp = TFMiniPlus::TFMP::new(sensor_serial).unwrap();

    loop {
        let (dist, strength, temp, buf) = tfmp.read().unwrap();
        println!("\r\n\r\nDist: {dist}\r\nStrength: {strength}\r\nTemp: {temp}\r\nBuf: {buf:?}\r\n");


        // let byte = rx.read();
        // match byte {
        //     Ok(byte) => println!("{byte:X}"),
        //     Err(err) => match err {
        //         nb::Error::Other(e) => println!("Error: {e}"),
        //         nb::Error::WouldBlock => thread::sleep(Duration::from_micros(10)),
        //     }   
        // }
    }
}

mod TFMiniPlus {
    use std::convert::TryInto;
    use std::thread;
    use std::time::Duration;

    use esp_idf_hal::adc::config::Resolution;
    use esp_idf_hal::serial::config::{StopBits, Parity, DataBits};
    use esp_idf_hal::serial::{Serial, Uart, Tx, Rx, SerialError};
    use esp_idf_hal::gpio::{InputPin, OutputPin};
    use esp_idf_sys::EspError;
    use embedded_hal::serial::Read;
    use embedded_hal::serial::Write;
    use nb::{block, Error};
    pub enum OutputFormat {
        CM,
        MM,
        Pixhawk,
    }

    #[derive(Debug)]
    pub enum TFMPError {
        NoResponse,
        BadResponse(Vec<u8>),
        BadChecksum(Vec<u8>),
        BadData,
        CommandFailed,
        SerialError(SerialError),
    }

    enum Command {
        GET_FIRMWARE_VERSION,
        SYSTEM_RESET,
        FRAME_RATE(u16),
        TRIGGER_DETECTION,
        OUTPUT_FORMAT(OutputFormat),
        BAUD_RATE(u32),
        ENABLE_OUTPUT,
        DISABLE_OUTPUT,
        RESET_FACTORY_SETTINGS,
        SAVE_SETTINGS,
    }

    enum Response {
        None,
        Version(u8, u8, u8),
        DataFrame((u16, u16, f32)),
        OutputFormat(OutputFormat),
        Framerate(u16),
        BaudRate(u32),
    }

    impl Command {
        pub fn get_bytes(&self) -> Vec<u8> {
            match self {
                Command::GET_FIRMWARE_VERSION => vec![0x5A, 0x04, 0x01, 0x5F],
                Command::SYSTEM_RESET => vec![0x5A, 0x04, 0x02, 0x60],
                Command::FRAME_RATE(frame_rate) => {
                    let mut output = vec![0x5A, 0x06, 0x03];
                    output.extend_from_slice(&frame_rate.to_le_bytes());
                    let sum = output.iter().sum::<u8>();
                    output.push(sum);
                    output
                },
                Command::TRIGGER_DETECTION => vec![0x5A, 0x04, 0x04, 0x62],
                Command::OUTPUT_FORMAT(format) => {
                    let mut output = vec![0x5A, 0x05, 0x05];
                    output.extend_from_slice(match format {
                        OutputFormat::CM => &[0x01, 0x65],
                        OutputFormat::MM => &[0x06, 0x6A],
                        OutputFormat::Pixhawk => &[0x02, 0x66],
                    });
                    output
                },
                Command::BAUD_RATE(rate) => {
                    let mut output = vec![0x5A, 0x08, 0x06];
                    output.extend_from_slice(&rate.to_le_bytes());
                    let sum = output.iter().sum::<u8>();
                    output.push(sum);
                    output
                },
                Command::ENABLE_OUTPUT => vec![0x5A, 0x05, 0x07, 0x01, 0x67],
                Command::DISABLE_OUTPUT => vec![0x5A, 0x05, 0x07, 0x00, 0x66],
                Command::RESET_FACTORY_SETTINGS => vec![0x5A, 0x04, 0x10, 0x6E],
                Command::SAVE_SETTINGS => vec![0x5A, 0x04, 0x11, 0x6F],
            }
        }

        pub fn get_response(&self, buf: Vec<u8>) -> Result<Response, TFMPError> {
            let sum = response.iter().sum::<u8>() - response[response.len() - 1];
            if sum != response[response.len() - 1] {
                return Err(TFMPError::BadChecksum(response));
            }
            match self {
                Command::GET_FIRMWARE_VERSION => {
                    let id = &response[0..=2];
                    if id == &[0x5A, 0x07, 0x01] {
                        return Ok(Response::Version(response[5], response[4], response[3]));
                    }
                },
                Command::SYSTEM_RESET => {
                    if response == &[0x5A, 0x05, 0x02, 0x00, 0x60] {
                        return Ok(Response::None);
                    } else if response == &[0x5A, 0x05, 0x02, 0x01, 0x61] {
                        return Err(TFMPError::CommandFailed);
                    }
                },
                Command::FRAME_RATE(_) => {
                    let id = &response[0..=2];
                    if id == &[0x5A, 0x06, 0x03] {
                        let set_rate = u16::from_le_bytes(response[3..=4].try_into().unwrap());
                        return Ok(Response::Framerate(set_rate));
                    }
                },
                Command::TRIGGER_DETECTION => {
                    let id = &response[0..=1];
                    if id == &[0x59, 0x59] {
                        return Ok(Response::DataFrame(read_data_frame(&response)));
                    }
                },
                Command::OUTPUT_FORMAT(_) => {
                    let id = &response[0..=2];
                    if id == &[0x5A, 0x05, 0x05] {
                        match &response[3..=4] {
                            &[0x01, 0x65] => return Ok(Response::OutputFormat(OutputFormat::CM)),
                            &[0x02, 0x66] => return Ok(Response::OutputFormat(OutputFormat::Pixhawk)),
                            &[0x06, 0x6A] => return Ok(Response::OutputFormat(OutputFormat::MM)),
                            _ => {}
                        }
                    }
                },
                Command::BAUD_RATE(_) => {
                    let id = &response[0..=2];
                    if id == &[0x5A, 0x08, 0x06] {
                        let baud_rate = u32::from_le_bytes(response[3..=6].try_into().unwrap());
                        return Ok(Response::BaudRate(baud_rate));
                    }
                },
                Command::ENABLE_OUTPUT => if response == &[0x5A, 0x05, 0x07, 0x01, 0x67] { return Ok(Response::None) },
                Command::DISABLE_OUTPUT => if response == &[0x5A, 0x05, 0x07, 0x00, 0x66] { return Ok(Response::None) },
                Command::RESET_FACTORY_SETTINGS => {
                    if response == &[0x5A, 0x05, 0x10, 0x00, 0x6E] {
                        return Ok(Response::None);
                    } else if response == &[0x5A, 0x05, 0x10, 0x01, 0x6F] {
                        return Err(TFMPError::CommandFailed);
                    }
                },
                Command::SAVE_SETTINGS => {
                    if response == &[0x5A, 0x05, 0x11, 0x00, 0x6F] {
                        return Ok(Response::None);
                    } else if response == &[0x5A, 0x05, 0x11, 0x01, 0x70] {
                        return Err(TFMPError::CommandFailed);
                    }
                },
            }
            // Nothing else worked
            Err(TFMPError::BadResponse(response))
        }
    }

    impl From<SerialError> for TFMPError {
        fn from(err: SerialError) -> Self {
            Self::SerialError(err)
        }
    }

    pub struct TFMP<UART: Uart> {
        tx: Tx<UART>,
        rx: Rx<UART>,
        framerate: u16,
        trigger_enabled: bool,
        output_format: OutputFormat,
        output_enabled: bool,
    }

    impl<UART: Uart> TFMP<UART> {
        pub fn new<TX: OutputPin, RX: InputPin>(mut serial: Serial<UART, TX, RX>) -> Result<Self, EspError> {
            // Configure Serial Lines
            // serial.change_baudrate(115200)?;
            // serial.change_data_bits(DataBits::DataBits8)?;
            // serial.change_stop_bits(StopBits::STOP1)?;
            // serial.change_parity(Parity::ParityNone)?;
            let (tx, rx) = serial.split();
            // Return Self
            Ok(Self {
                tx,
                rx,
                framerate: 100,
                trigger_enabled: false,
                output_format: OutputFormat::CM,
                output_enabled: true,
            })
        }

        /// Returns seperated data from sensor in format:
        /// (Distance 0-1200, Strength 0-65535, Temp in Celsius)
        pub fn read(&mut self) -> Result<(u16, u16, f32, Vec<u8>), TFMPError> {
            let mut i = 0;
            loop {
                // Attempt to find first byte
                for _ in 0..10 { if self.read_byte()? == 0x59 { break }}
                // Read second magic byte
                if self.read_byte()? == 0x59 { break };
                // Increment fail counter if it doesn't work
                i += 1;
                if i > 10 {
                    return Err(TFMPError::BadData);
                }
            }
            // We've found the start of a byte
            // So read all data into an array for checksum checking
            let mut buf = vec![0x59u8; 9];
            let mut checksum = 0x59 + 0x59;
            for i in 2..buf.len() {
                buf[i] = self.read_byte()?;
                if i < buf.len() - 1 {
                    checksum += buf[i] as u16;
                }
            }
            // Check checksum
            if checksum as u8 != buf[buf.len() - 1] {
                return Err(TFMPError::BadChecksum(buf))
            }
            let (dist, strength, temp) = read_data_frame(&buf);
            // Return values
            Ok((dist, strength, temp, buf))
        }

        fn send_command(&mut self, command: Command) -> Result<[u8; 9], TFMPError> {
            let data = command.get_bytes();
            for byte in data {
                block!(self.tx.write(byte))?;
            }
            command.get_response(&mut self.rx);
            Ok([0; 9])
        }

        fn flush_read_buffer(&mut self) -> Result<Vec<u8>, TFMPError> {
            let output = vec![];
            while self.rx.count().unwrap() > 0 {
                output.push(self.read_byte()?);
            }
            Ok(output)
        }

        fn read_byte(&mut self) -> Result<u8, TFMPError> {
            let byte = {
                let mut i = 1;
                loop {
                    match self.rx.read() {
                        Ok(val) => break val,
                        Err(Error::WouldBlock) => {},
                        Err(Error::Other(err)) => return Err(err.into()),
                    }
                    i += 1;
                    if i >= 1000 {
                        return Err(TFMPError::NoResponse);
                    }
                    thread::sleep(Duration::from_millis(1));
                }
            };
            Ok(byte)
        }
    }

    fn read_data_frame(buf: &Vec<u8>) -> (u16, u16, f32) {
        // Start reading distance data
        let dist = (buf[2] as u16) | ((buf[3] as u16) << 8);
        // Next, read Strength
        let strength = (buf[4] as u16) | ((buf[5] as u16) << 8);
        // Finally, read and convert temp to Celsius
        let temp = ((buf[6] as u16) | ((buf[7] as u16) << 8)) as f32 / 8. - 256.;
        //
        (dist, strength, temp)
    }

}
