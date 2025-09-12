use crate::foc::EDir;
use crate::foc::MAX_MOTOR_NR;
use crate::EFocSimpleError;
use crate::Result;
use fixed::types::I16F16;
use rtt_target::rprintln;

// lookup table from the raw hall state (1-3-2-6-4-5  as index to get the hall state )
//                                  0  1  2  3  4  5  6  7
const HALL_STATE_TABLE: [usize; 8] = [0, 0, 2, 1, 4, 5, 3, 0];
pub struct FocHallSensor {
  motor_nr: usize,
  velocity: f32, // rotation speed in rad/microsecond
  angle: I16F16,
  angle_per_state: I16F16, // == TAU / (nr_poles * 6)
  dir: EDir,
  state_prev: usize,
  error_count: usize,
  hall_idx_max: usize,  // nr_poles times 6
  hall_idx_base: usize, // increments each time with 6 until the hall_idx_max
}

impl FocHallSensor {
  pub fn new(motor_nr: usize, nr_poles: usize) -> Self {
    let hall_idx_max = nr_poles * 6;
    let angle_per_state = I16F16::TAU / hall_idx_max as i32;
    if motor_nr >= MAX_MOTOR_NR {
      rprintln!("Incorrect motor number. Panic now");
      panic!();
    }
    FocHallSensor {
      motor_nr,
      dir: EDir::Stopped,
      state_prev: 0,
      error_count: 0,
      velocity: 0.0,
      angle: I16F16::ZERO,
      angle_per_state,
      hall_idx_base: 0,
      hall_idx_max,
    }
  }

  pub fn get_errorcount(&self) -> usize {
    self.error_count
  }
  pub fn reset_errorcount(&mut self) {
    self.error_count = 0;
  }

  /// update the angle of the shaft with the hall state
  /// incoming raw state is the hall sensor state in the first 3 bits. Valid transitions: 1, 3, 2 ,6 ,4 ,5

  pub fn update(&mut self, raw_state: usize) -> Result<I16F16> {
    if raw_state == 0 || raw_state > 6 {
      self.error_count += 1;
      rprintln!("Error Invalid hall state {}", raw_state,);
      Err(EFocSimpleError::AngleSensorError)
    } else {
      let prev = self.state_prev;
      let state = HALL_STATE_TABLE[raw_state];
      self.state_prev = state;

      match state {
        0 => {
          if prev == 5 {
            self.hall_idx_base += 6;
            if self.hall_idx_base >= self.hall_idx_max {
              self.hall_idx_base = 0;
            }
            self.dir = EDir::Cw;
          } else if prev != 1 {
            rprintln!("Error change new:{} old:{}", state, prev);
            self.error_count += 1
          }
        }
        5 => {
          if prev == 0 {
            if self.hall_idx_base < 6 {
              self.hall_idx_base = self.hall_idx_max - 6;
            } else {
              self.hall_idx_base -= 6;
            }
          } else if prev != 4 {
            rprintln!("Error change new:{} old:{}", state, prev);
            self.error_count += 1
          }
        }
        _ => {
          if state.abs_diff(prev) != 1 {
            rprintln!("Error change new:{} old:{}", state, prev);
            self.error_count += 1
          }
        }
      }
      let hall_state_idx = self.hall_idx_base + state;
      // calculate the not interpolated angle
      let angle = self.angle_per_state * hall_state_idx as i32;

      Ok(angle)
    }
  }
}
