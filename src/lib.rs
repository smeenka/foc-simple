#![no_std]
#![allow(dead_code)]
use core::future::Future;

use fixed::types::I16F16;

pub use crate::tools::foc_command::FocCommand;
use crate::tools::shaft_position::ShaftPosition;

pub mod foc;
pub mod tools;

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum EFocCommand {
  FocMode(foc::EFocMode),
  Speed(I16F16),
  SpeedAcc(I16F16),
  Angle(I16F16),
  Torque(I16F16),
  ShaftPosition(ShaftPosition),
  TorqueLimit(I16F16),
  ErrorCount,
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum EFocSimpleError {
  Uninitialized,
  NoMagWarning,
  DataError,
  SerialError,
  AngleSensorError,
  MotorNrError,
  NoMotorMovement,
  NoAngleSensor,
  NoCurrentSensor,
  NotImplementedYet,
  NoData,
  Error,
}
pub type Result<T> = core::result::Result<T, EFocSimpleError>;

/// The trait for getting the shaft angle of the motor from some sensor
pub trait SensorAngle {
  /// Retrieve the angle from the sensor, in a synchrounous way
  /// Return on success the raw angle from the sensor, in radians from 0 .. 2 pi
  fn get_angle(&mut self) -> Result<I16F16>;
}

#[derive(Clone, Debug, Copy)]
pub enum EFocAngleSensor<A> {
  SensorLess,
  ShaftSensor(A), // contains the implementation of the SensorAngle trait
  HallSensor,
}

#[derive(Clone, Debug, Copy)]
pub enum EFocCurrentSensor<C> {
  SensorLess,
  ThreePhase(C), // contains the implementation of the SensorCurrent Trait
}

#[derive(Clone, Debug, Copy)]
pub enum EFocAngle {
  SensorLess,
  SensorValue(I16F16), // actual measured sensor value
  Interpolate,         // interpolate actual value
}

// The trait for getting the current from the motor
pub trait SensorCurrent {
  /// Retrieve the current from the sensor.
  /// Return on success the current array from the sensor, in amperes
  /// In case 2 phases are sample set the third phase to zero here
  fn get_current(&mut self) -> Result<[I16F16; 3]>;
}

pub type MessageRx = [u8; 64];
pub struct MessageTx {
  pub data: [u8; 64],
  pub len: usize,
}

pub trait FocSerial {
  fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize>>;
  fn send(&mut self, buffer: &[u8]) -> Result<()>;
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub struct FocParam {
  p: I16F16,
  i: I16F16,
  d: I16F16,
}
impl FocParam {
  pub fn new_fp(fp: f32, fi: f32, fd: f32) -> Self {
    FocParam {
      p: I16F16::from_num(fp),
      i: I16F16::from_num(fi),
      d: I16F16::from_num(fd),
    }
  }
  pub fn new(p: I16F16, i: I16F16, d: I16F16) -> Self {
    FocParam { p, i, d }
  }
  pub fn set_p(&mut self, p: I16F16) {
    self.p = p
  }
  pub fn set_i(&mut self, i: I16F16) {
    self.i = i
  }
  pub fn set_d(&mut self, d: I16F16) {
    self.d = d
  }
  pub fn get_p(&mut self) -> I16F16 {
    self.p
  }
  pub fn get_i(&mut self) -> I16F16 {
    self.i
  }
  pub fn get_d(&mut self) -> I16F16 {
    self.d
  }
}
