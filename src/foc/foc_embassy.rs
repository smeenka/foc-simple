use crate::foc::COMMAND_CHANNEL_SIZE;
use crate::foc::MAX_MOTOR_NR;
use embassy_futures::select::{select4, Either4};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Instant, Ticker};
use fixed::types::I16F16;
use rtt_target::rprintln;

use crate::{
  foc::{foc_pwm::FocPwm, foc_simple::FocSimple, PwmDriver},
  EFocAngle, EFocAngleSensor, EFocCommand, EFocCurrentSensor, SensorAngle, SensorCurrent,
};

static CHANNEL_COMMANDS: [Channel<CriticalSectionRawMutex, EFocCommand, COMMAND_CHANNEL_SIZE>; MAX_MOTOR_NR] =
  [Channel::new(), Channel::new()];
pub static HALL_ANGLES: [Signal<CriticalSectionRawMutex, I16F16>; MAX_MOTOR_NR] = [Signal::new(), Signal::new()];

pub struct FocEmbassy<A: SensorAngle, C: SensorCurrent, D: PwmDriver> {
  foc: FocSimple,
  foc_pwm: FocPwm<D>,
  motor_nr: usize,
  // angle sensor
  angle_sensor: EFocAngleSensor<A>,
  current_sensor: EFocCurrentSensor<C>,

  hs_ticker: Ticker, // high speed ticker for the foc calculations
  ls_ticker: Ticker, // low speed ticker for the velocity and the user queues
  error_count: usize,
  hall_angle: I16F16,
}

impl<A, C, D> FocEmbassy<A, C, D>
where
  A: SensorAngle,
  C: SensorCurrent,
  D: PwmDriver,
{
  pub fn new(
    motor_nr: usize,
    foc: FocSimple,
    foc_pwm: FocPwm<D>,
    hs_ticker: Ticker,
    ls_ticker: Ticker,
  ) -> FocEmbassy<A, C, D> {
    FocEmbassy {
      motor_nr,
      foc,
      foc_pwm,
      hs_ticker,
      ls_ticker,

      angle_sensor: EFocAngleSensor::SensorLess,
      current_sensor: EFocCurrentSensor::SensorLess,

      error_count: 0,
      hall_angle: I16F16::ZERO,
    }
  }
  /// set the sensor with an actual implementation of AngleSensor
  pub fn set_angle_sensor(&mut self, sensor: EFocAngleSensor<A>) {
    self.angle_sensor = sensor
  }
  pub fn set_current_sensor(&mut self, sensor: EFocCurrentSensor<C>) {
    self.current_sensor = sensor
  }

  pub async fn task(&mut self) {
    rprintln!("Start foc update task");
    if self.motor_nr >= MAX_MOTOR_NR {
      rprintln!("Incorrect motor number {}. Aborting", self.motor_nr);
      return;
    }

    loop {
      match select4(
        HALL_ANGLES[self.motor_nr].wait(),
        self.hs_ticker.next(),
        self.ls_ticker.next(),
        CHANNEL_COMMANDS[self.motor_nr].receive(),
      )
      .await
      {
        Either4::First(a) => self.hall_angle = a,
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
            EFocAngleSensor::HallSensor => EFocAngle::SensorValue(self.hall_angle),
          };
          match self.foc.update(angle) {
            Ok((angle, torque)) => _ = self.foc_pwm.update(angle, torque, None),
            Err(_e) => {
              self.error_count += 1
              // handle error
            }
          }
        }
        Either4::Third(_) => {
          let now = Instant::now().as_millis() as usize;
          self.foc.update_velocity(now);
        }
        Either4::Fourth(command) => match command {
          EFocCommand::FocMode(mode) => _ = self.foc.set_foc_mode(mode),
          EFocCommand::ShaftPosition(shaft_pos) => _ = self.foc.set_position_req(shaft_pos),
          EFocCommand::Speed(speed) => self.foc.set_speed(speed),
          EFocCommand::SpeedAcc(acc) => self.foc.set_acceleration(acc),
          EFocCommand::Torque(t) => self.foc.set_torque(t),
          EFocCommand::Angle(a) => self.foc.set_angle(a),
          EFocCommand::TorqueLimit(tl) => _ = self.foc_pwm.set_torque_limit(tl),
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
