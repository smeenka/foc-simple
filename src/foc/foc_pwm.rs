#[allow(dead_code)]
use fixed::types::I16F16;
use foc::park_clarke::{self, TwoPhaseReferenceFrame};

use crate::foc::{EModulation, PwmDriver};
use crate::tools::foc_pid::FocPd;
use crate::{EFocSimpleError, Result};

pub struct FocPwm<D: PwmDriver> {
  driver: D,
  modulation: EModulation,
  max_pwm: u16,
  max_torque: I16F16, // can be set at in the enum foc_mode. Must be in range 0.01..1
  torque_limit_neg: I16F16,
  torque_limit_pos: I16F16,
  torque_pd: FocPd,
  flux_pd: FocPd,
}

impl<D> FocPwm<D>
where
  D: PwmDriver,
{
  pub fn new(driver: D, modulation: EModulation, max_pwm: u16, max_torque: f32) -> Self {
    let max_torque = I16F16::from_num(max_torque.clamp(0.0, 1.0));
    FocPwm {
      modulation,
      driver,
      max_pwm,
      max_torque,
      torque_limit_neg: -1 * max_torque,
      torque_limit_pos: max_torque,
      torque_pd: FocPd::default(),
      flux_pd: FocPd::default(),
    }
  }

  pub fn set_torque_limit(&mut self, torque: I16F16) {
    let torque = torque.clamp(I16F16::ZERO, self.max_torque);
    self.torque_limit_neg = -1 * torque;
    self.torque_limit_pos = torque;
  }

  /// update the actual duty cycle value of the pwm drivers, based on the given torque and flush and electrical angle
  /// If the current is given, calculate the torque and the flux with the pd controllers, else pass the torque as is
  // v_q: desired torque, v_d: desired flux
  pub fn update(&mut self, angle: I16F16, torque: I16F16, current: Option<(I16F16, I16F16, I16F16)>) -> Result<()> {
    let (sin_angle, cos_angle) = cordic::sin_cos(angle);
    match current {
      None => self.update_sin_cos(sin_angle, cos_angle, torque, I16F16::ZERO),
      Some(_) => Err(EFocSimpleError::NotImplementedYet),
    }
  }

  /// update the actual duty cycle value of the pwm drivers, based on the given sin and cos and the torque and flush
  // v_q: desired torque, v_d: desired flux
  pub fn update_sin_cos(&mut self, sin_angle: I16F16, cos_angle: I16F16, v_q: I16F16, v_d: I16F16) -> Result<()> {
    let torque = v_q.clamp(self.torque_limit_neg, self.torque_limit_pos);
    // Inverse Park transform
    let orthogonal_voltage = park_clarke::inverse_park(
      cos_angle,
      sin_angle,
      park_clarke::RotatingReferenceFrame { d: v_d, q: torque },
    );
    // convert to the pwm values, in range -1 .. 1
    let vectors = self.modulate(orthogonal_voltage);
    // convert to range 1 .. max_pwm value for setting the duty cycles on the timers
    let duties = vectors.map(|val| {
      (((val + I16F16::ONE) * (self.max_pwm as i32 + 1)) / 2)
        .round()
        .saturating_to_num::<u16>()
        .clamp(0, self.max_pwm)
    });
    // and set duty_cyles on the timers, as implemented by the user
    self.driver.set_pwm(duties)
  }

  /// Returns a value between -1 and 1 for each channel.
  /// The content of this function is copied from the foc library, v0.3.0
  /// because it was impossible to reuse the code
  fn modulate(&self, value: TwoPhaseReferenceFrame) -> [I16F16; 3] {
    match self.modulation {
      EModulation::Sinusoidal => {
        let voltages = park_clarke::inverse_clarke(value);
        [voltages.a, voltages.b, voltages.c]
      }
      EModulation::SpaceVector => {
        // Convert alpha/beta to x/y/z
        let sqrt_3_alpha = I16F16::SQRT_3 * value.alpha;
        let beta = value.beta;
        let x = beta;
        let y = (beta + sqrt_3_alpha) / 2;
        let z = (beta - sqrt_3_alpha) / 2;

        // Calculate which sector the value falls in
        let sector: u8 = match (x.is_positive(), y.is_positive(), z.is_positive()) {
          (true, true, false) => 1,
          (_, true, true) => 2,
          (true, false, true) => 3,
          (false, false, true) => 4,
          (_, false, false) => 5,
          (false, true, false) => 6,
        };

        // Map a,b,c values to three phase
        let (ta, tb, tc);
        match sector {
          1 | 4 => {
            ta = x - z;
            tb = x + z;
            tc = -x + z;
          }
          2 | 5 => {
            ta = y - z;
            tb = y + z;
            tc = -y - z;
          }
          3 | 6 => {
            ta = y - x;
            tb = -y + x;
            tc = -y - x;
          }
          _ => unreachable!("invalid sector"),
        }

        [ta, tb, tc]
      }
      EModulation::Square => {
        let voltages = park_clarke::inverse_clarke(value);

        [voltages.a.signum(), voltages.b.signum(), voltages.c.signum()]
      }
      EModulation::Trapezoidal => {
        let voltages = park_clarke::inverse_clarke(value);
        [
          (voltages.a * 2).round_to_zero().signum(),
          (voltages.b * 2).round_to_zero().signum(),
          (voltages.c * 2).round_to_zero().signum(),
        ]
      }
    }
  }
}
