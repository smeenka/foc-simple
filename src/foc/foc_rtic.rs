use crate::foc::foc_pwm::FocPwm;
use crate::foc::COMMAND_CHANNEL_SIZE;
use crate::foc::MAX_MOTOR_NR;
use embassy_futures::select::{select4, Either4};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use fixed::types::I16F16;
use rtt_target::rprintln;

use crate::{
  foc::{foc_simple::FocSimple, PwmDriver},
  EFocAngle, EFocAngleSensor, EFocCommand, SensorAngle, SensorCurrent,
};

static CHANNEL_COMMANDS: [Channel<CriticalSectionRawMutex, EFocCommand, COMMAND_CHANNEL_SIZE>; MAX_MOTOR_NR] = [Channel::new(), Channel::new()];
pub static HALL_ANGLES: [Signal<CriticalSectionRawMutex, I16F16>; MAX_MOTOR_NR] = [Signal::new(), Signal::new()];
pub static SIGNAL_1MS: [Signal<CriticalSectionRawMutex, ()>; MAX_MOTOR_NR] = [Signal::new(), Signal::new()];
pub static SIGNAL_10MS: [Signal<CriticalSectionRawMutex, usize>; MAX_MOTOR_NR] = [Signal::new(), Signal::new()];

pub struct FocRtic<A: SensorAngle, C: SensorCurrent, D: PwmDriver> {
  motor_nr: usize,
  foc: FocSimple,
  foc_pwm: FocPwm<D>,
  // angle sensor
  angle_sensor: EFocAngleSensor<A>,
  current_sensor: Option<C>,
  error_count: usize,
}

impl<A, C, D> FocRtic<A, C, D>
where
  A: SensorAngle,
  C: SensorCurrent,
  D: PwmDriver,
{
  pub fn new(motor_nr: usize, foc: FocSimple, foc_pwm: FocPwm<D>) -> FocRtic<A, C, D> {
    if motor_nr > MAX_MOTOR_NR {
      rprintln!("Incorrect motor number {}. Aborting", motor_nr);
    }
    FocRtic {
      motor_nr,
      foc,
      foc_pwm,
      angle_sensor: EFocAngleSensor::SensorLess,
      current_sensor: None,
      error_count: 0,
    }
  }
  /// set the sensor with an actual implementation of AngleSensor
  pub fn set_angle_sensor(&mut self, sensor: EFocAngleSensor<A>) {
    self.angle_sensor = sensor
  }
  pub fn set_current_sensor(&mut self, sensor: Option<C>) {
    self.current_sensor = sensor
  }
  pub fn get_foc(&mut self) -> &mut FocSimple {
    &mut self.foc
  }

  pub async fn task(&mut self) {
    let nr = self.motor_nr;
    loop {
      match select4(HALL_ANGLES[nr].wait(), SIGNAL_1MS[nr].wait(), SIGNAL_10MS[nr].wait(), CHANNEL_COMMANDS[nr].receive()).await {
        Either4::First(a) => {
          match self.foc.update(EFocAngle::SensorValue(a)) {
            Ok((angle, torque)) => {
              if let Err(_) = self.foc_pwm.update(angle, torque, None) {
                self.error_count += 1
                // handle error
              }
            }
            Err(_e) => {
              self.error_count += 1
              // handle error
            }
          }
        }
        Either4::Second(_) => {
          let angle = match &mut self.angle_sensor {
            EFocAngleSensor::SensorLess => EFocAngle::SensorLess,
            EFocAngleSensor::ShaftSensor(s) => match s.get_angle() {
              Err(_) => {
                rprintln!("Error while getting shaft angle");
                // reuse previous value
                EFocAngle::SensorValue(self.foc.get_position_act().get_angle())
              }
              Ok(a) => EFocAngle::SensorValue(a),
            },
            EFocAngleSensor::HallSensor => EFocAngle::Interpolate,
          };

          match self.foc.update(angle) {
            Ok((angle, torque)) => {
              if let Err(_) = self.foc_pwm.update(angle, torque, None) {
                self.error_count += 1
                // handle error
              }
            }
            Err(_e) => {
              self.error_count += 1
              // handle error
            }
          }
        }
        Either4::Third(now) => self.foc.update_velocity(now),
        Either4::Fourth(command) => match command {
          EFocCommand::FocMode(mode) => {
            _ = self.foc.set_foc_mode(mode);
            ()
          }
          EFocCommand::ShaftPosition(shaft_pos) => self.foc.set_position_req(shaft_pos),
          EFocCommand::Speed(speed) => self.foc.set_speed(speed),
          EFocCommand::SpeedAcc(acc) => self.foc.set_acceleration(acc),
          EFocCommand::Torque(t) => self.foc.set_torque(t),
          EFocCommand::Angle(a) => self.foc.set_angle(a),
          EFocCommand::TorqueLimit(tl) => self.foc_pwm.set_torque_limit(tl),
          EFocCommand::ErrorCount => {
            rprintln!("Total error count:{}", self.error_count);
            self.error_count = 0
          }
        },
      }
    }
  }
}

pub struct FocSimpleCommand {}

impl FocSimpleCommand {
  pub fn send_command(idx: usize, command: EFocCommand) {
    if idx < MAX_MOTOR_NR {
      if let Err(_e) = CHANNEL_COMMANDS[idx].try_send(command) {
        rprintln!("Command channel full for motor {}", idx);
      }
    }
  }
}
