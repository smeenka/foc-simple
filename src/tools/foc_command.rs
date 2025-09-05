use core::fmt::Write;
use fixed::types::I16F16;
use heapless::String;

#[cfg(feature = "embassy")]
use crate::foc::foc_embassy::FocSimpleCommand;
#[cfg(feature = "rtic")]
use crate::foc::foc_rtic::FocSimpleCommand;

use crate::{
  foc::{CalParams, EDir, EFocMode},
  EFocCommand, FocParam, FocSerial,
};

#[derive(Clone, Debug, Copy, PartialEq)]
#[repr(usize)]
enum EFocModeLocal {
  Idle = 0,
  Calibration,
  Velocity,
  Angle,
  Torque,
}

pub struct FocCommand<C: FocSerial> {
  // user request
  serial: C,
  motor_nr: usize,
  foc_mode: EFocModeLocal,
  param_v: FocParam,
  param_a: FocParam,
  param_t: FocParam,
  param_c: Option<CalParams>,
}

impl<C> FocCommand<C>
where
  C: FocSerial,
{
  pub fn new(serial: C) -> Self {
    let param_v = FocParam::new_fp(0.1, 0.0, 0.0);
    let param_a = FocParam::new_fp(0.4, 0.0, 0.0);
    let param_t = FocParam::new_fp(1.0, 0.0, 0.0);
    FocCommand {
      serial,
      foc_mode: EFocModeLocal::Idle,
      param_a,
      param_t,
      param_v,
      param_c: None,
      motor_nr: 0,
    }
  }
  /// the task will become the owner of Self
  pub async fn task(&mut self) {
    self.send("\r\nStarting Foc Serial Parser task\r\n");
    self.help();
    let mut line = [0_u8; 128];
    let mut line_idx = 0;
    loop {
      let mut word = [0_u8; 32];
      if let Ok(len) = self.serial.receive(&mut word).await {
        for idx in 0..len {
          let a = word[idx];
          if a == 10 || a == 13 || a == 32 {
            self.handle_line(&line[0..line_idx]);
            line_idx = 0;
          } else {
            line[line_idx] = a;
            line_idx += 1;
          }
        }
      }
    }
  }

  fn handle_line(&mut self, line: &[u8]) {
    let gotmsg: &str = core::str::from_utf8(line).unwrap().into();
    let words = gotmsg.split(" ");
    for word in words {
      self.handle_word(word);
    }
  }
  fn handle_word(&mut self, m: &str) {
    if m.len() < 2 {
      self.send("Word must be at least 2 characters\r\n");
      return;
    }
    match &m[0..2] {
      "he" => self.help(),
      "ts" => self.set_speed(m, "Speed:"),
      "ta" => self.set_angle(m, "Angle:"),
      "tt" => self.set_torque(m, "Torque:"),
      "tl" => self.set_torque_limit(m, "Torque Limit"),
      "pa" => self.set_acceleration(m, "Speed Acceleration:"),
      "mi" => self.select_mode(EFocModeLocal::Idle),
      "mc" => self.select_mode(EFocModeLocal::Calibration),
      "mv" => self.select_mode(EFocModeLocal::Velocity),
      "ma" => self.select_mode(EFocModeLocal::Angle),
      "mt" => self.select_mode(EFocModeLocal::Torque),
      "m0" => self.select_motor(0),
      "m1" => self.select_motor(1),
      "m2" => self.select_motor(2),
      "kp" => self.set_param(&m),
      "ki" => self.set_param(&m),
      "kd" => self.set_param(&m),
      "co" => self.set_calibration_offset(&m),
      "ec" => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::ErrorCount),
      _ => self.help(),
    }
  }

  fn send(&mut self, message: &str) {
    let bytes = message.as_bytes();
    _ = self.serial.send(bytes);
  }

  fn send_nl(&mut self) {
    let bytes = "\r\n".as_bytes();
    _ = self.serial.send(bytes);
  }

  pub fn send_usize(&mut self, data: usize) {
    let mut msg: String<10> = String::new();
    core::write!(&mut msg, "{}", data).unwrap();
    self.send(&msg);
  } /*
    pub fn send_I16F16(&mut self, data: I16F16) {
      let mut msg: String<20> = String::new();
      core::write!(&mut msg, "{}", data).unwrap();
      self.send(&msg);
    }*/
  pub fn send_i16f16(&mut self, data: I16F16) {
    let mut msg: String<20> = String::new();
    //let data: f32 = data.to_num();
    core::write!(&mut msg, "{}", data).unwrap();
    self.send(&msg);
  }

  fn select_mode(&mut self, mode: EFocModeLocal) {
    self.send("Select mode:");
    self.foc_mode = mode;
    match mode {
      EFocModeLocal::Angle => self.send("Angle\r\n"),
      EFocModeLocal::Velocity => self.send("Velocity\r\n"),
      EFocModeLocal::Torque => self.send("Torque\r\n"),
      EFocModeLocal::Calibration => self.send("Calibration\r\n"),
      EFocModeLocal::Idle => self.send("Idle\r\n"),
    };
    match mode {
      EFocModeLocal::Angle => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Angle(self.param_a))),
      EFocModeLocal::Velocity => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Velocity(self.param_v))),
      EFocModeLocal::Torque => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Torque(self.param_t))),
      EFocModeLocal::Calibration => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Calibration(None))),
      EFocModeLocal::Idle => FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Idle)),
    };
  }
  fn select_motor(&mut self, m: usize) {
    self.send("Select motor:");
    self.send_usize(m);
    self.send_nl();
    self.motor_nr = m;
  }
  fn set_speed(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      FocSimpleCommand::send_command(self.motor_nr, EFocCommand::Speed(f));
    }
  }
  fn set_angle(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      FocSimpleCommand::send_command(self.motor_nr, EFocCommand::Angle(f));
    }
  }
  fn set_torque(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      FocSimpleCommand::send_command(self.motor_nr, EFocCommand::Torque(f));
    }
  }
  fn set_acceleration(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      FocSimpleCommand::send_command(self.motor_nr, EFocCommand::SpeedAcc(f));
    }
  }

  fn set_torque_limit(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      FocSimpleCommand::send_command(self.motor_nr, EFocCommand::TorqueLimit(f));
    }
  }


  fn parse_float(&mut self, word: &str, text: &str) -> Option<I16F16> {
    self.send(text);
    match word[2..].parse::<I16F16>() {
      Err(_) => {
        self.send("Unable to parse float after command\r\n");
        None
      }
      Ok(f) => {
        self.send_i16f16(f);
        self.send_nl();
        Some(f)
      }
    }
  }

  fn set_param(&mut self, word: &str) {
    match word[2..].parse::<I16F16>() {
      Err(_) => self.send("Unable to parse float after command\r\n"),
      Ok(f) => {
        let mode = match self.foc_mode {
          EFocModeLocal::Angle => "Angle",
          EFocModeLocal::Velocity => "Velocity",
          EFocModeLocal::Torque => "Torque",
          _ => "Invalid, choose mode first",
        };
        let p = match self.foc_mode {
          EFocModeLocal::Angle => &mut self.param_a,
          EFocModeLocal::Velocity => &mut self.param_v,
          EFocModeLocal::Torque => &mut self.param_t,
          _ => &mut FocParam::new(I16F16::ZERO, I16F16::ZERO, I16F16::ZERO),
        };
        match &word[0..2] {
          "kp" => p.set_p(f),
          "ki" => p.set_i(f),
          "kd" => p.set_d(f),
          _ => (),
        };
        self.send("Set pid parameters for mode:");
        self.send(mode);
        self.send(" ");
        self.send(&word[0..2]);
        self.send(" = ");
        self.send_i16f16(f);
        self.send_nl();
      }
    }
  }

  fn set_calibration_offset(&mut self, word: &str) {
    match word[2..].parse::<I16F16>() {
      Err(_) => self.send("Unable to parse float after command\r\n"),
      Ok(f) => {
        self.send("Set electrical offset to:");
        self.send_i16f16(f);
        self.send_nl();
        self.param_c = Some(CalParams::new(EDir::Cw, I16F16::from_num(f)));
        FocSimpleCommand::send_command(self.motor_nr, EFocCommand::FocMode(EFocMode::Calibration(self.param_c.clone())));
      }
    }
  }

  pub fn help(&mut self) {
    self.send("\r\nHow to use  ... \r\n");
    self.send("  he        -- help this message\r\n");
    self.send("  ts<float> -- set target speed\r\n");
    self.send("  ta<float> -- set target angle\r\n");
    self.send("  tt<float> -- set target torque\r\n");
    self.send("  tl<float> -- set max torque limit\r\n");
    self.send("  pa<float> -- set speed acceleration in rad/sec2\r\n");
    self.send("  mi        -- mode idle\r\n");
    self.send("  mv        -- mode velocity\r\n");
    self.send("  mt        -- mode torque\r\n");
    self.send("  m0        -- select motor 0\r\n");
    self.send("  m1        -- select motor 1\r\n");
    self.send("  co        -- calibration offset. \r\n");
    self.send("  pi<0|1>   -- enable/disable interpolation. \r\n");
    self.send("  kp<float> -- pid P\r\n");
    self.send("  ki<float> -- pid I\r\n");
    self.send("  kd<float> -- pid D\r\n");
    self.send("  ec        -- Show error count on rtt channel and reset\r\n");
  }
}
