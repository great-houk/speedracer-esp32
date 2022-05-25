#![allow(dead_code)]
use std::rc::Rc;

use esp_idf_hal::{gpio::OutputPin, ledc::{HwTimer, HwChannel, config::TimerConfig, Timer, Channel}};
use esp_idf_hal::prelude::*;
use esp_idf_sys::EspError;

type Duty = u32;
const FREQ: u32 = 50;
const MIN: Duty = 1000;
const MAX: Duty = 2000;
const MAX_ANGLE: i32 = 90;

const TOTAL_LEN: f32 = (1_000 * 1_000) as f32 / FREQ as f32;

#[derive(Debug)]
pub enum ServoError {
    BadAngle,
    BadPulseLen,
    EspError(EspError),
}

impl From<EspError> for ServoError {
    fn from(err: EspError) -> Self {
        Self::EspError(err)
    }
}

pub struct Servo<C: HwChannel, H: HwTimer, P: OutputPin> {
    channel: Channel<C, H, Rc<Timer<H>>, P>,
    duty: u32,
}

impl<C: HwChannel, H: HwTimer, P: OutputPin> Servo<C, H, P> {
    pub fn new(timer: H, channel: C, pin: P) -> Result<Self, ServoError> {
        let config = TimerConfig::default().frequency(FREQ.Hz().into()).resolution(esp_idf_hal::ledc::Resolution::Bits15);
        let timer = Rc::new(Timer::new(timer, &config)?);
        let mut channel = Channel::new(channel, timer, pin)?;
        let duty = Self::from_degrees(0, &channel)?;
        channel.set_duty(duty)?;
        Ok(Self {
            channel,
            duty,
        })
    }

    pub fn get_pulse_len(&self) -> u32 {
        Self::to_us(self.duty, &self.channel)
    }

    pub fn get_angle(&self) -> i32 {
        Self::to_degrees(self.duty, &self.channel)
    }

    pub fn set_us(&mut self, pulse_len: u32) -> Result<(), ServoError> {
        let duty = Self::from_us(pulse_len, &self.channel);
        self.channel.set_duty(duty)?;
        self.duty = duty;
        Ok(())
    }

    pub fn set_degrees(&mut self, angle: i32) -> Result<(), ServoError> {
        let duty = Self::from_degrees(angle, &self.channel)?;
        self.channel.set_duty(duty)?;
        self.duty = duty;
        Ok(())
    }

    fn to_degrees(duty: Duty, channel: &Channel<C, H, Rc<Timer<H>>, P>) -> i32 {
        let length = Self::to_us(duty, channel) - MIN;
        let ratio = length as f32 / (MAX - MIN) as f32;
        let angle = (180. * ratio) - 90.;
        angle.round() as i32
    }

    fn from_degrees(angle: i32, channel: &Channel<C, H, Rc<Timer<H>>, P>) -> Result<Duty, ServoError> {
        if angle > MAX_ANGLE || angle < -MAX_ANGLE {
            return Err(ServoError::BadAngle);
        }
        let percent = (angle + MAX_ANGLE) as f32 / (MAX_ANGLE * 2) as f32;
        let length = ((MAX - MIN) as f32 * percent).round() as u32 + MIN;
        Ok(Self::from_us(length, channel))
    }

    fn to_us(duty: u32, channel: &Channel<C, H, Rc<Timer<H>>, P>) -> Duty {
        let ratio = duty as f32 / channel.get_max_duty() as f32;
        (ratio * TOTAL_LEN as f32).round() as Duty
    }

    fn from_us(pulse_length: u32, channel: &Channel<C, H, Rc<Timer<H>>, P>) -> Duty {
        ((pulse_length as f32 / TOTAL_LEN) * channel.get_max_duty() as f32).round() as Duty
    }
}