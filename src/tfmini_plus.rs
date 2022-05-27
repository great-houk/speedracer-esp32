#![allow(dead_code)]
use embedded_hal::prelude::*;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{InputPin, OutputPin};
use esp_idf_hal::serial::config::{DataBits, FlowControl, Parity, StopBits};
use esp_idf_hal::serial::{SerialError, Uart};
use esp_idf_sys::{uart_is_driver_installed, EspError};
use nb::{block, Error};
use serial_util::*;
use std::sync::{Arc, Mutex, MutexGuard};

type MilliSeconds = u32;
const TIMEOUT: MilliSeconds = 250;

#[derive(Debug, PartialEq, Clone)]
pub struct FrameData {
    pub dist: u16,
    pub strength: u16,
    pub temp: u16,
    pub raw: Vec<u8>,
}

impl Default for FrameData {
    fn default() -> Self {
        Self {
            dist: 0,
            strength: 0,
            temp: 0,
            raw: vec![],
        }
    }
}

impl From<(u16, u16, u16, Vec<u8>)> for FrameData {
    fn from((dist, strength, temp, raw): (u16, u16, u16, Vec<u8>)) -> Self {
        Self {
            dist,
            strength,
            temp,
            raw,
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum OutputFormat {
    CM,
    MM,
    Pixhawk,
}

#[derive(Debug)]
pub enum TFMPError {
    NoResponse,
    WrongResponse(Vec<u8>),
    BadChecksum(Vec<u8>),
    BadData,
    CommandFailed,
    SerialError(SerialError),
    EspError(EspError),
    NotInTriggerMode,
    OutputDisabled,
    WrongMethod,
}

impl From<SerialError> for TFMPError {
    fn from(err: SerialError) -> Self {
        Self::SerialError(err)
    }
}

impl From<EspError> for TFMPError {
    fn from(err: EspError) -> Self {
        Self::EspError(err)
    }
}

enum Command {
    GetFirmwareVersion,
    SystemReset,
    FrameRate(u16),
    TriggerDetection,
    OutputFormat(OutputFormat),
    BaudRate(u32),
    EnableOutput(bool),
    ResetFactorySettings,
    SaveSettings,
}

impl Command {
    pub fn get_bytes(&self) -> Vec<u8> {
        match self {
            Command::GetFirmwareVersion => vec![0x5A, 0x04, 0x01, 0x5F],
            Command::SystemReset => vec![0x5A, 0x04, 0x02, 0x60],
            Command::FrameRate(frame_rate) => {
                let mut output = vec![0x5A, 0x06, 0x03];
                output.extend_from_slice(&frame_rate.to_le_bytes());
                let checksum = checksum(&output);
                output.push(checksum);
                output
            }
            Command::TriggerDetection => vec![0x5A, 0x04, 0x04, 0x62],
            Command::OutputFormat(format) => {
                let mut output = vec![0x5A, 0x05, 0x05];
                output.extend_from_slice(match format {
                    OutputFormat::CM => &[0x01, 0x65],
                    OutputFormat::MM => &[0x06, 0x6A],
                    OutputFormat::Pixhawk => &[0x02, 0x66],
                });
                output
            }
            Command::BaudRate(rate) => {
                let mut output = vec![0x5A, 0x08, 0x06];
                output.extend_from_slice(&rate.to_le_bytes());
                let checksum = checksum(&output);
                output.push(checksum);
                output
            }
            Command::EnableOutput(enabled) => {
                vec![0x5A, 0x05, 0x07, if *enabled { 1 } else { 0 }, 0x67]
            }
            Command::ResetFactorySettings => vec![0x5A, 0x04, 0x10, 0x6E],
            Command::SaveSettings => vec![0x5A, 0x04, 0x11, 0x6F],
        }
    }

    pub fn get_response<UART: Uart>(
        &self,
        serial: &mut Serial<UART>,
    ) -> Result<Response, TFMPError> {
        match self {
            Command::TriggerDetection => self.receive_frame(serial),
            _ => self.receive_response(serial),
        }
    }

    fn receive_frame<UART: Uart>(&self, serial: &mut Serial<UART>) -> Result<Response, TFMPError> {
        // Find response
        const RESPONSE_LEN: usize = 9;
        const RESPONSE_START: [u8; 2] = [0x59, 0x59];
        let response = read_data_frame(serial, &RESPONSE_START, RESPONSE_LEN)?;
        // Check checksum
        let sum = checksum(&response[0..8]);
        if sum != response[8] {
            return Err(TFMPError::BadChecksum(response));
        }
        // Analyze Response
        return Ok(Response::DataFrame(response));
    }

    fn receive_response<UART: Uart>(
        &self,
        serial: &mut Serial<UART>,
    ) -> Result<Response, TFMPError> {
        // Get supposed length and ID
        let (len, id) = match self {
            Command::GetFirmwareVersion => (0x07, 0x01),
            Command::SystemReset => (0x05, 0x02),
            Command::FrameRate(_) => (0x06, 0x03),
            Command::TriggerDetection => unreachable!(),
            Command::OutputFormat(_) => (0x05, 0x05),
            Command::BaudRate(_) => (0x08, 0x06),
            Command::EnableOutput(_) => (0x05, 0x07),
            Command::ResetFactorySettings => (0x05, 0x10),
            Command::SaveSettings => (0x05, 0x11),
        };
        let first_three_bytes = [0x5A, len, id];
        // Find message with that length and starting byte
        let response = read_data_frame(serial, &first_three_bytes, len as usize)?;
        // Check checksum
        let sum = checksum(&response[..response.len() - 1]);
        if sum != response[response.len() - 1] {
            return Err(TFMPError::BadChecksum(response));
        }
        // Interpret Response
        match self {
            Command::GetFirmwareVersion => {
                Ok(Response::Version(response[5], response[4], response[3]))
            }
            Command::FrameRate(_) => Ok(Response::Framerate(u16::from_le_bytes([
                response[3],
                response[4],
            ]))),
            Command::TriggerDetection => unreachable!(),
            Command::OutputFormat(_) => Ok(Response::OutputFormat(match response[3] {
                0x01 => OutputFormat::CM,
                0x02 => OutputFormat::Pixhawk,
                0x06 => OutputFormat::MM,
                _ => unreachable!(),
            })),
            Command::BaudRate(_) => Ok(Response::BaudRate(u32::from_le_bytes([
                response[3],
                response[4],
                response[6],
                response[6],
            ]))),
            Command::EnableOutput(_) => Ok(Response::OutputEnabled(if response[3] == 0 {
                false
            } else {
                true
            })),
            Command::SystemReset | Command::ResetFactorySettings | Command::SaveSettings => {
                if response[3] == 0 {
                    Ok(Response::None)
                } else {
                    Err(TFMPError::CommandFailed)
                }
            }
        }
    }
}

enum Response {
    None,
    Version(u8, u8, u8),
    DataFrame(Vec<u8>),
    OutputFormat(OutputFormat),
    OutputEnabled(bool),
    Framerate(u16),
    BaudRate(u32),
}

pub struct TFMP<UART: Uart> {
    uart: Arc<Mutex<UART>>,
    tx: i32,
    rx: i32,
    framerate: u16,
    baudrate: u32,
    output_format: OutputFormat,
    output_enabled: bool,
}

// Contains Useful Commands
impl<UART: Uart> TFMP<UART> {
    pub fn new<TX: OutputPin, RX: InputPin>(
        uart: Arc<Mutex<UART>>,
        tx: TX,
        rx: RX,
    ) -> Result<Self, EspError> {
        use esp_idf_sys::{uart_config_t, uart_driver_install, uart_param_config};

        // Configure Serial Lines
        let uart_config = uart_config_t {
            baud_rate: 115_200i32,
            data_bits: DataBits::DataBits8.into(),
            parity: Parity::ParityNone.into(),
            stop_bits: StopBits::STOP1.into(),
            flow_ctrl: FlowControl::None.into(),
            ..Default::default()
        };

        EspError::convert(unsafe { uart_param_config(UART::port(), &uart_config) }).unwrap();

        const UART_FIFO_SIZE: i32 = 128;
        if !unsafe { uart_is_driver_installed(UART::port()) } {
            EspError::convert(unsafe {
                uart_driver_install(
                    UART::port(),
                    UART_FIFO_SIZE * 2,
                    UART_FIFO_SIZE * 2,
                    0,
                    core::ptr::null_mut(),
                    0,
                )
            })
            .unwrap();
        }

        Ok(Self {
            uart,
            tx: tx.pin(),
            rx: rx.pin(),
            framerate: 100,
            baudrate: 115200,
            output_format: OutputFormat::CM,
            output_enabled: true,
        })
    }

    fn save_changes(&mut self) -> Result<Response, TFMPError> {
        let response = self.send_command(Command::SaveSettings);
        // Weird behaviour where the save command won't return anything
        response
        // match response {
        //     Err(TFMPError::NoResponse) => Ok(Response::None),
        //     _ => response
        // }
    }

    fn send_command(&mut self, command: Command) -> Result<Response, TFMPError> {
        // Get command to send
        let data = command.get_bytes();
        // Get serial
        let mut serial = self.get_serial();
        // Flush read
        serial.flush_rx()?;
        // Send the actual data
        for byte in data {
            block!(serial.write(byte))?;
        }
        block!(serial.flush())?;
        // Check what the response was
        let result = command.get_response(&mut serial);
        // Pause for a bit
        // FreeRtos.delay_ms(1u32);
        result
    }

    fn interpret_data_frame(&self, buf: Vec<u8>) -> Result<FrameData, TFMPError> {
        // Check checksum
        let checksum = checksum(&buf[..8]);
        if checksum != buf[8] {
            return Err(TFMPError::BadChecksum(buf));
        }
        // Extraxt values
        let (dist, strength, temp) = {
            // Start reading distance data
            let dist = (buf[2] as u16) | ((buf[3] as u16) << 8);
            // Next, read Strength
            let strength = (buf[4] as u16) | ((buf[5] as u16) << 8);
            // Finally, read and convert temp to Celsius
            let temp = ((buf[6] as u16) | ((buf[7] as u16) << 8)) / 8 - 256;
            // Return
            (dist, strength, temp)
        };
        // Return values
        Ok((dist, strength, temp, buf).into())
    }

    pub fn get_serial(&self) -> Serial<UART> {
        Serial::new(self.uart.lock().unwrap(), self.tx, self.rx)
    }
}

// Contains user facing commands
impl<UART: Uart> TFMP<UART> {
    /// Returns seperated data from sensor in format:
    /// (Distance 0-1200, Strength 0-65535, Temp in Celsius)
    pub fn read(&mut self) -> Result<FrameData, TFMPError> {
        if self.output_format == OutputFormat::Pixhawk {
            return Err(TFMPError::WrongMethod);
        }
        // Check if we can expect a frame
        if self.output_enabled == false {
            return Err(TFMPError::OutputDisabled);
        }
        // Get incoming frame
        let mut serial = self.get_serial();
        let frame = read_data_frame(&mut serial, &[0x59, 0x59], 9)?;
        // Read data
        let data = self.interpret_data_frame(frame);
        data
    }

    /// Reads the output of the sensor, if the format is in pixhawk.
    /// The formating is so different, that this needs a new function.
    /// The returned string has no newline
    pub fn read_pixhawk(&mut self) -> Result<String, TFMPError> {
        if self.output_format != OutputFormat::Pixhawk {
            return Err(TFMPError::WrongMethod);
        }
        // Check if we can expect a frame
        if self.output_enabled == false {
            return Err(TFMPError::OutputDisabled);
        }
        // let time = Instant::now();
        // Create buffer to store upcoming frame
        let mut buf = Vec::with_capacity(6);
        // Sanity check
        let mut i = 0;
        let mut serial = self.get_serial();
        // println!("Init Read in {}", time.elapsed().as_micros());
        // let time = Instant::now();
        // Read until we find 13, 10, which are \r\n for this thing
        while buf.len() < 2 || &buf[buf.len() - 2..] != &['\r' as u8, '\n' as u8] {
            let byte = read_byte(&mut serial)?;
            buf.push(byte);
            // Just in case...
            if i > 100 {
                return Err(TFMPError::BadData);
            }
            i += 1;
        }
        // // println!("Read output in {}", time.elapsed().as_micros());
        // Return output minus the \r\n
        Ok(String::from_utf8_lossy(&buf[..buf.len() - 2]).to_string())
    }

    /// Returns the firmware version of the sensor, in the format (V3, V2, V1) for V3.V2.V1
    pub fn get_firmware_version(&mut self) -> Result<(u8, u8, u8), TFMPError> {
        if let Response::Version(a, b, c) = self.send_command(Command::GetFirmwareVersion)? {
            Ok((a, b, c))
        } else {
            unreachable!()
        }
    }

    /// Resets the sensor
    pub fn system_reset(&mut self) -> Result<(), TFMPError> {
        self.send_command(Command::SystemReset)?;
        Ok(())
    }

    /// Change the framerate of the sensor.
    /// Set to zero to make the sensor a oneshot, triggered by the trigger command
    pub fn set_framerate(&mut self, fps: u16) -> Result<u16, TFMPError> {
        if let Response::Framerate(fps) = self.send_command(Command::FrameRate(fps))? {
            self.save_changes()?;
            self.framerate = fps;
            Ok(fps)
        } else {
            unreachable!();
        }
    }

    /// Triggers the sensor to capture, given it is in trigger mode (The framerate is zero)
    pub fn trigger(&mut self) -> Result<FrameData, TFMPError> {
        if self.framerate != 0 {
            return Err(TFMPError::NotInTriggerMode);
        }
        if let Response::DataFrame(response) = self.send_command(Command::TriggerDetection)? {
            let data = self.interpret_data_frame(response)?;
            Ok(data)
        } else {
            unreachable!()
        }
    }

    /// Puts the sensor in trigger mode. The same as setting the framerate to zero
    pub fn into_trigger_mode(&mut self) -> Result<(), TFMPError> {
        let fps = self.set_framerate(0)?;
        if fps != 0 {
            Err(TFMPError::CommandFailed)
        } else {
            Ok(())
        }
    }

    /// Sets the output format of the sensor. Can be: CM, Pixart (Decimal String in M), MM
    pub fn set_output_format(&mut self, output: OutputFormat) -> Result<OutputFormat, TFMPError> {
        if let Response::OutputFormat(format) = self.send_command(Command::OutputFormat(output))? {
            self.save_changes()?;
            self.output_format = format.clone();
            Ok(format)
        } else {
            unreachable!()
        }
    }

    /// Sets the baudrate for the sensor to communicate with. Unsafe, because the sensor only supports standard rates,
    /// and there aren't any checks built into this function.
    pub unsafe fn set_baudrate(&mut self, rate: u32) -> Result<u32, TFMPError> {
        if let Response::BaudRate(rate) = self.send_command(Command::BaudRate(rate))? {
            self.save_changes()?;
            Ok(rate)
        } else {
            unreachable!()
        }
    }

    /// Sets the output for the sensor to be enabled or disabled.
    pub fn enable_output(&mut self, enabled: bool) -> Result<(), TFMPError> {
        if let Response::OutputEnabled(acc_enabled) =
            self.send_command(Command::EnableOutput(enabled))?
        {
            self.save_changes()?;
            if acc_enabled != enabled {
                return Err(TFMPError::CommandFailed);
            }
            Ok(())
        } else {
            unreachable!()
        }
    }

    /// Restores the factory settings for the sensor
    pub fn reset_factory_settings(&mut self) -> Result<(), TFMPError> {
        self.send_command(Command::ResetFactorySettings)?;
        self.save_changes()?;
        self.framerate = 100;
        self.baudrate = 115200;
        self.output_enabled = true;
        self.output_format = OutputFormat::CM;
        Ok(())
    }
}

mod serial_util {
    use super::*;

    pub struct Serial<'a, UART: Uart> {
        _uart: MutexGuard<'a, UART>,
    }

    impl<'a, UART: Uart> Serial<'a, UART> {
        pub fn new(uart: MutexGuard<'a, UART>, tx: i32, rx: i32) -> Self {
            use esp_idf_sys::{uart_flush_input, uart_read_bytes, uart_set_pin};

            EspError::convert(unsafe { uart_flush_input(UART::port()) }).unwrap();

            EspError::convert(unsafe { uart_set_pin(UART::port(), tx, rx, -1, -1) }).unwrap();

            let mut buf = [0u8; 10];
            unsafe {
                uart_read_bytes(
                    UART::port(),
                    &mut buf as *mut u8 as *mut _,
                    buf.len() as u32,
                    0,
                )
            };

            EspError::convert(unsafe { uart_flush_input(UART::port()) }).unwrap();

            // FreeRtos.delay_us(250u32);

            Self { _uart: uart }
        }

        pub fn flush_rx(&mut self) -> Result<(), EspError> {
            use esp_idf_sys::uart_flush_input;
            EspError::convert(unsafe { uart_flush_input(UART::port()) })
        }

        pub fn read(&mut self) -> nb::Result<u8, SerialError> {
            use esp_idf_sys::{uart_read_bytes, ESP_ERR_INVALID_STATE};

            // Code copied from esp_idf_hal::serial::Rx::read
            let mut buf: u8 = 0;

            // uart_read_bytes() returns error (-1) or how many bytes were read out
            // 0 means timeout and nothing is yet read out
            match unsafe { uart_read_bytes(UART::port(), &mut buf as *mut u8 as *mut _, 1, 0) } {
                1 => Ok(buf),
                0 => Err(nb::Error::WouldBlock),
                _ => Err(nb::Error::Other(SerialError::other(
                    EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
                ))),
            }
        }

        pub fn count(&self) -> Result<u8, EspError> {
            use esp_idf_sys::uart_get_buffered_data_len;

            // Code copied from esp_idf_hal::serial::Rx::count
            let mut size = 0_u32;

            EspError::check_and_return(
                unsafe { uart_get_buffered_data_len(UART::port(), &mut size) },
                size as u8,
            )
        }

        pub fn flush(&mut self) -> nb::Result<(), SerialError> {
            use esp_idf_sys::{uart_wait_tx_done, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT, ESP_OK};

            // Code copied from esp_idf_hal::serial::Tx::flush
            match unsafe { uart_wait_tx_done(UART::port(), 0) } {
                ESP_OK => Ok(()),
                ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
                _ => Err(nb::Error::Other(SerialError::other(
                    EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
                ))),
            }
        }

        pub fn write(&mut self, byte: u8) -> nb::Result<(), SerialError> {
            use esp_idf_sys::{uart_write_bytes, ESP_ERR_INVALID_STATE};

            // Code copied from esp_idf_hal::serial::Tx::flush
            // `uart_write_bytes()` returns error (-1) or how many bytes were written
            match unsafe { uart_write_bytes(UART::port(), &byte as *const u8 as *const _, 1) } {
                1 => Ok(()),
                _ => Err(nb::Error::Other(SerialError::other(
                    EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
                ))),
            }
        }
    }

    pub fn read_data_frame(
        serial: &mut Serial<impl Uart>,
        pattern: &[u8],
        len: usize,
    ) -> Result<Vec<u8>, TFMPError> {
        // Attempt to find pattern
        let mut i = 0;
        const MAX_ITER: i32 = 20;
        'attempting: loop {
            i += 1;
            if i >= MAX_ITER {
                return Err(TFMPError::NoResponse);
            }
            // Go through all bytes in pattern, and try again if any don't match
            for &byte in pattern {
                if read_byte(serial)? != byte {
                    continue 'attempting;
                }
            }
            // We found all of the pattern bytes,
            // so continue through the program.
            break 'attempting;
        }

        // We've found the start of a pattern
        // So read all data into an array
        let mut buf = Vec::from(pattern);
        for _ in buf.len()..len {
            buf.push(read_byte(serial)?);
        }
        Ok(buf)
    }

    pub fn read_byte(serial: &mut Serial<impl Uart>) -> Result<u8, TFMPError> {
        let byte = {
            let mut i = 1;
            let b = loop {
                match serial.read() {
                    Ok(val) => break val,
                    Err(Error::WouldBlock) => {}
                    Err(Error::Other(err)) => return Err(err.into()),
                }
                i += 1;
                if i >= TIMEOUT * 100 {
                    return Err(TFMPError::NoResponse);
                }
                FreeRtos.delay_us(10u32);
            };
            b
        };
        Ok(byte)
    }

    pub fn checksum(buf: &[u8]) -> u8 {
        buf.iter().fold(0, |acc, x| acc.wrapping_add(*x))
    }
}
