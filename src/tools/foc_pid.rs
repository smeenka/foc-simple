use fixed::types::I16F16;

use crate::tools::shaft_position::ShaftPosition;


/// A fixed-point PID controller.
/// note that the time element is omitted. The delta time is very constant in the foc controller
/// So this pid is only usable for this foc controller.
pub struct FocPid {
  kp: I16F16,
  ki: I16F16,
  kd: I16F16,

  integral: I16F16,
  integral_max_p: I16F16,
  integral_max_n: I16F16,

  last_measurement: I16F16,
  last_position: ShaftPosition,
  ready: bool
}

impl FocPid {
  pub fn default() -> Self {
    FocPid::new(I16F16::ONE, I16F16::ZERO, I16F16::ZERO)
  }
  /// Create a new PID controller with the given gains.
  pub fn new(kp: I16F16, ki: I16F16, kd: I16F16) -> Self {
    Self {
      kp,
      ki,
      integral: I16F16::ZERO,
      kd,
      last_measurement: I16F16::ZERO,
      last_position: ShaftPosition::new(),
      integral_max_n: -1 * I16F16::ONE,
      integral_max_p: 1 * I16F16::ONE,
      ready: false
    }
  }

  pub fn set_integral_max(&mut self, max: I16F16) {
    self.integral_max_p = max;
    self.integral_max_n = -1 * max;
  }

  pub fn reset_integral(&mut self) {
    self.integral = I16F16::ZERO
  }

  /// Update the PID controller, returning the new output value.
  pub fn update(&mut self, setpoint: I16F16, measurement: I16F16) -> I16F16 {
    let error = setpoint - measurement ;
    let p = self.kp * error;

    let integral = (self.integral + error).clamp(self.integral_max_n, self.integral_max_p);
    self.integral = integral;

    let i = self.ki * integral;

    let delta_m = if self.ready {
      measurement - self.last_measurement
    } else {
      self.ready = true;
      I16F16::ZERO
    };
    self.last_measurement = measurement;

    let d = self.kd * delta_m;

    p + i - d
  }


  pub fn update_position(&mut self, setpoint: &ShaftPosition, measurement: &ShaftPosition) -> I16F16 {
    let error = setpoint.compare(measurement);
    let p = self.kp * error;

    let integral = (self.integral + error).clamp(self.integral_max_n, self.integral_max_p);
    self.integral = integral;

    let i = self.ki * integral;

    let delta_m = if self.ready {
      measurement.compare(&self.last_position)
    } else {
      self.ready = true;
      I16F16::ZERO
    };
    self.last_position.rotations = measurement.rotations;
    self.last_position.angle = measurement.angle;

    let d = self.kd * delta_m;

    p + i - d
  }


}


/// A fixed-point PD controller.
/// note that the time element is omitted. The delta time is very constant in the foc controller
/// This pd controller should be used for the torque and flux controllers
/// So this pd is only usable for the pid controller
pub struct FocPd {
  kp: I16F16,
  kd: I16F16,
  last_measurement: I16F16,
  ready: bool
}

impl FocPd {
  /// Create a new PID controller with the given gains.
  pub fn default() -> Self {
    FocPd::new(I16F16::ONE, I16F16::ZERO)
  }
  pub fn new(kp: I16F16, kd: I16F16) -> Self {
    Self {
      kp,
      kd,
      last_measurement: I16F16::ZERO,
      ready: false
    }
  }


  /// Update the PID controller, returning the new output value.
  pub fn update(&mut self, setpoint: I16F16, measurement: I16F16) -> I16F16 {
    let error = setpoint - measurement ;
    let p = self.kp * error;


    let delta_m = if self.ready {
      measurement - self.last_measurement
    } else {
      self.ready = true;
      I16F16::ZERO
    };
    self.last_measurement = measurement;

    let d = self.kd * delta_m;

    p - d
  }
}
