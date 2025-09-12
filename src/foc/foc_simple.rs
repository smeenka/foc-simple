use fixed::types::I16F16;
use rtt_target::rprintln;

use crate::{
  foc::{EDir, EFocMode},
  tools::foc_pid::FocPid,
  EFocAngle, EFocSimpleError, Result, ShaftPosition,
};

pub struct FocSimple {
  // user request
  shaft_position_req: ShaftPosition,
  foc_mode: EFocMode,
  calibration_state: ECalibrateState,
  torque: I16F16, // target for torque in %
  target_pid: FocPid,
  // angle sensor
  shaft_position_act: ShaftPosition,
  velocity: I16F16, // rad / s. Filtered with a lowpass filter of ca 10 Hrz
  nr_poles: usize,
  // current sensor
  // internal state
  electrical_offset: I16F16, // offset of the angle sensor with respect to the poles of the motor in radians.
  timestamp_vel: usize,
  speed_req: I16F16,
  speed_acc: I16F16, // in rad/100 ms
  speed_act: I16F16,
  temp: I16F16,
}

#[derive(Debug)]
#[repr(u8)]
enum ECalibrateState {
  Init = 0,
  FindDirection,
  FindOffset,
  ReturnToStart,
}

impl FocSimple {
  pub fn new(nr_poles: usize) -> FocSimple {
    FocSimple {
      // user request parameters
      shaft_position_req: ShaftPosition::new(),
      torque: I16F16::ZERO,
      foc_mode: EFocMode::Idle,
      electrical_offset: I16F16::ZERO,
      velocity: I16F16::ZERO, // in rad per second
      shaft_position_act: ShaftPosition::new(),
      nr_poles,
      // current sensor
      // internal state
      calibration_state: ECalibrateState::Init,
      target_pid: FocPid::new(I16F16::ONE, I16F16::ONE, I16F16::ONE),
      timestamp_vel: 0,
      temp: I16F16::ZERO,
      speed_req: I16F16::ZERO,
      speed_acc: I16F16::ONE / 10, // in rad/100 ms
      speed_act: I16F16::ZERO,
    }
  }

  /// Velocity mode: Set the speed in rad/sec, positive or negative
  /// Speed wil increment or decrement until this target value is reached
  pub fn set_speed(&mut self, speed: I16F16) {
    self.speed_req = speed;
  }
  /// Torque mode: set the torque in range -1 ..1
  /// Calibration mode: Set the torque for calibration
  /// Torque is set immediatly
  pub fn set_torque(&mut self, torque: I16F16) {
    self.torque = torque;
  }
  /// Angle mode: set the angle in range 0 .. TAU
  /// Angle wil be set immediatly. Speed of change can be regulated with torque limit
  pub fn set_angle(&mut self, angle: I16F16) {
    if let EFocMode::Angle(_) = self.foc_mode {
      self.shaft_position_req.angle = angle;
    }
  }
  /// Angle mode: set the position with shaft position. Can be positive or negative
  /// Angle wil be set immediatly. Speed of change can be regulated with torque limit
  pub fn set_position(&mut self, position: ShaftPosition) {
    if let EFocMode::Angle(_) = self.foc_mode {
      self.shaft_position_req = position;
    }
  }
  /// acceleration in rad/sec2
  pub fn set_acceleration(&mut self, acc: I16F16) {
    self.speed_acc = acc / 100; // update is done with 100 hrz
  }
  /// only to be used for test function. Be carefull with setting the value manually!
  /// Incorrect use can damage the motor.
  pub fn set_electrical_offset(&mut self, offset: I16F16) {
    self.electrical_offset = offset;
  }

  /// return the busy flag, to see if the calibration is finished, or the target angle is reached
  pub fn is_idle(&self) -> bool {
    self.foc_mode == EFocMode::Idle
  }

  pub fn get_velocity(&self) -> I16F16 {
    self.velocity
  }
  pub fn get_position_act(&self) -> ShaftPosition {
    self.shaft_position_act
  }
  pub fn get_position_req(&self) -> ShaftPosition {
    self.shaft_position_req
  }
  pub fn set_position_req(&mut self, req: ShaftPosition) {
    self.shaft_position_req = req;
  }

  pub fn set_foc_mode(&mut self, mode: EFocMode) -> Result<()> {
    rprintln!("Set Foc mode");
    self.foc_mode = mode;
    match self.foc_mode {
      EFocMode::Calibration(param) => match param {
        Some(p) => {
          self.electrical_offset = p.zero;
          self.shaft_position_act.set_inversed(p.dir == EDir::Ccw);
          self.foc_mode = EFocMode::Idle;
        }
        None => {
          self.electrical_offset = I16F16::ZERO;
          self.shaft_position_act.set_inversed(false);
          self.calibration_state = ECalibrateState::Init;
          if self.torque < I16F16::ONE/10 {
            self.torque = I16F16::ONE/4;
          }
        }
      },
      EFocMode::Angle(param) => {
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE * 3);

        self.shaft_position_req = self.shaft_position_act.clone();
      }
      EFocMode::Velocity(param) => {
        self.speed_req = I16F16::ZERO;
        self.speed_act = I16F16::ZERO;
        self.shaft_position_req = self.shaft_position_act.clone();
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE * 20);
      }
      EFocMode::Torque(param) => {
        self.torque = I16F16::ZERO;
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE);
      }
      EFocMode::Idle => (),
      EFocMode::Error(e) => return Err(e),
    }
    Ok(())
  }

  /// update the state of the foc controller. as fast as possible. Proposed value is each ms
  /// Input is the measured shaft angle in radians
  /// Details of how to get the angle are not in scope here, so no dependencies towards hardware
  /// Returned is a tuple of the electrical angle, and the requested torque
  pub fn update(&mut self, angle: EFocAngle) -> Result<(I16F16, I16F16)> {
    let electrical_angle = match angle {
      EFocAngle::SensorLess => I16F16::ZERO,
      EFocAngle::SensorValue(angle) => {
        self.shaft_position_act.update_shaft_angle(angle);
        I16F16::from_num(self.nr_poles) * self.shaft_position_act.angle - self.electrical_offset
      }
      EFocAngle::Interpolate => {
        I16F16::from_num(self.nr_poles) * self.shaft_position_act.angle - self.electrical_offset
      }
    };
    match self.foc_mode {
      EFocMode::Idle => Ok((electrical_angle, I16F16::ZERO)),
      EFocMode::Error(e) => Err(e),
      EFocMode::Calibration(_) => match angle {
        EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
        _ => self.do_calibration(),
      },

      EFocMode::Angle(_) => {
        match angle {
          EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
          _ => {
            // Compare actual position with requested
            let torque = self
              .target_pid
              .update_position(&self.shaft_position_req, &self.shaft_position_act);
            Ok((electrical_angle, torque))
          }
        }
      }
      EFocMode::Velocity(_) => {
        match angle {
          EFocAngle::SensorLess => {
            // note that in sensorless mode the shaft_position_req is the electrical angle of the shaft
            let delta_electrical_angle = I16F16::from_num(self.nr_poles) * self.speed_act / 1000;
            self.shaft_position_req.inc(delta_electrical_angle); // increment requested angle in rad/s
                                                                 // set the torque with the torque limit function
            let request_angle = self.shaft_position_req.get_angle();
            Ok((request_angle, I16F16::ONE / 4))
          }
          _ => {
            // increment the requested shaft position, but only if the diff with the actual shaft position is not too big
            let diff = self.shaft_position_req.compare(&self.shaft_position_act);
            if diff.abs() < I16F16::PI {
              let delta_angle = self.speed_act / 1000;
              self.shaft_position_req.inc(delta_angle);
            }
            let requested_torque = self
              .target_pid
              .update_position(&self.shaft_position_req, &self.shaft_position_act);
            Ok((electrical_angle, requested_torque))
          }
        }
      }
      EFocMode::Torque(_) => match angle {
        EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
        _ => Ok((electrical_angle, self.torque)),
      },
    }
  }

  /// calculate the velocity. This function should be called each 10 ms.
  /// The low pass filter frequency is 10 hrz
  pub fn update_velocity(&mut self, ts: usize) {
    // update the actual requested speed  with a fixed frequency of preferable 100 hz
    self.update_speed();
    let delta_ts = ts - self.timestamp_vel;
    if delta_ts > 0 {
      self.timestamp_vel = ts;
      let delta_sec = I16F16::from_num(delta_ts) / 1_000;
      let position_delta = self.shaft_position_act.delta();
      let velocity_current = position_delta / delta_sec; // in rad per second
                                                         // filter the velocity with a low pass filter

      self.velocity = (velocity_current + 19 * self.velocity) / 20;
    }
  }

  #[inline]
  fn update_speed(&mut self) {
    let req = self.speed_req;
    if self.speed_acc == 0 {
      self.speed_act = req;
    } else {
      let mut act = self.speed_act;
      if act > req {
        act -= self.speed_acc;
        if act < req {
          act = req;
        }
      } else if act < req {
        act += self.speed_acc;
        if act > req {
          act = req;
        }
      } // do nothing if equal
      self.speed_act = act;
    }
  }

  /// Calculate the direction of the sensor in relation to the direction of the motor
  /// If needed invert the direction of the sensor
  /// Returned is a tuple of electrical angle and torque
  fn do_calibration(&mut self) -> Result<(I16F16, I16F16)> {
    let req = self.shaft_position_req;
    let act = self.shaft_position_act;

    match self.calibration_state {
      ECalibrateState::Init => {
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.electrical_offset = I16F16::ZERO;
        self.calibration_state = ECalibrateState::FindDirection
      }
      ECalibrateState::FindDirection => {
        // state end condition after exact 1 electrical turn
        if req.rotations > 1 {
          // check motor did move
          if act.rotations == 0 && act.angle == I16F16::ZERO {
            self.calibration_state = ECalibrateState::Init;
            self.foc_mode = EFocMode::Error(EFocSimpleError::NoMotorMovement);
            rprintln!("End condition No motor movement detected");
          } else {
            // set the direction
            if act.get_position() < 0 {
              // reverse the direction in the driver. Motor must run in the same dir as the sensor
              rprintln!("Direction inversed");
              self.shaft_position_act.set_inversed(true);
            } else {
              rprintln!("Direction not inversed");
              self.shaft_position_act.set_inversed(false);
            }

            self.calibration_state = ECalibrateState::FindOffset;
          }
        } else {
          // rotate with 10 rad/sec positive
          self.shaft_position_req.inc(I16F16::from_num(0.01));
        }
      }
      ECalibrateState::FindOffset => {
        // state end condition after exact 2 electrical turns + 3/4 TAU
        if req.rotations > 2 && req.angle > 3 * I16F16::FRAC_TAU_4 {
          // determin electrical offset with current offset == 0
          let offset = self.shaft_position_act.get_angle() * self.nr_poles as i32;
          // normalize to 0 .. TAU
          self.electrical_offset = ShaftPosition::clamp(offset);
          self.calibration_state = ECalibrateState::ReturnToStart;
          rprintln!("Electrical offset:{}", self.electrical_offset);
        } else {
          // rotate with 10 rad/sec positive
          self.shaft_position_req.inc(I16F16::from_num(0.01));
        }
      }
      ECalibrateState::ReturnToStart => {
        // end conditions at start positon plu TAU/2
        if req.rotations == 0 && req.angle < I16F16::FRAC_TAU_2 {
          self.foc_mode = EFocMode::Idle;
          rprintln!("Calibraton finished");
        } else {
          // rotate with 10 rad/sec negative
          self.shaft_position_req.inc(I16F16::from_num(-0.01));
        }
      }
    }
    Ok((self.shaft_position_req.get_angle(), self.torque.abs()))
  }
}
