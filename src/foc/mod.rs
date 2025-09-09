pub mod foc_hall_sensor;
pub mod foc_pwm;
pub mod foc_simple;

#[cfg(feature = "embassy")]
pub mod foc_embassy;
#[cfg(feature = "rtic")]
pub mod foc_rtic;

pub const MAX_MOTOR_NR: usize = 2;
pub const COMMAND_CHANNEL_SIZE: usize = 25;

use fixed::types::I16F16;

use crate::{EFocSimpleError, FocParam, Result};

#[derive(Clone, Debug, Copy)]
pub enum EModulation {
  /// Generate PWM values based on a sinusoidal waveform.
  ///
  /// While this method is very simple (and fast) it is less efficient than SpaceVector
  /// as it does not utilise the bus voltage as well.
  Sinusoidal,
  /// Generate PWM values based on a trapezoidal wave.
  ///
  /// Note that for this method to work properly, when the output is 0 the
  /// resective channel should be disabled/set as high impedance.
  Trapezoidal,
  /// Generate PWM values based on a square wave.
  Square,
  SpaceVector,
}

pub trait PwmDriver {
  /// set the pwm duty cycles for A B and C.
  fn set_pwm(&mut self, duty_cycles: [u16; 3]) -> Result<()>;
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum EDir {
  Stopped,
  Cw,
  Ccw,
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub struct CalParams {
  dir: EDir,
  zero: I16F16,
}
impl CalParams {
  pub fn new(dir: EDir, zero: I16F16) -> Self {
    CalParams { dir, zero }
  }
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum EFocMode {
  Idle,
  Calibration(Option<CalParams>),
  Velocity(FocParam),
  Angle(FocParam),
  Torque(FocParam),
  Error(EFocSimpleError),
}
