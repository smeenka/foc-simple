//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::foc_simple::FocSimple;
use foc_simple::foc::EModulation;
use foc_simple::{EFocAngle, FocParam};
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};
use storm32::foc_impl::{LeftDriverImpl, RightDriverImpl};

bind_interrupts!(
  struct Irqs {}
);

#[embassy_executor::task]
pub async fn ticker_task(mut led: Output<'static>) {
  let mut ticker = Ticker::every(Duration::from_millis(300));
  loop {
    led.toggle();
    ticker.next().await;
  }
}
#[embassy_executor::task]
pub async fn left_motor_task(mut foc_pwm: FocPwm<LeftDriverImpl<'static>>) {
  rprintln!("Left motor  only used for angle sensor measurement");
  // left is running in full foc mode, right is running sensorless
  let mut foc = FocSimple::new(7);
  let mut ticker = Ticker::every(Duration::from_millis(1));

  let parm = FocParam::new_fp(2.0, 0.00, 0.1);
  _ = foc.set_foc_mode(foc_simple::foc::EFocMode::Velocity(parm));
  foc.set_speed(I16F16::TAU);
  loop {
    // 2. Run the FOC algorithm
    let now = Instant::now().as_millis() as usize;
    match foc.update(EFocAngle::SensorLess) {
      Err(_) => {
        rprintln!("Error while updating foc");
      }
      Ok((angle, torque)) => foc_pwm.update(angle,torque, None).unwrap(),
    }
    foc.update_velocity(now);
    ticker.next().await;
  }
}
#[embassy_executor::task]
pub async fn right_motor_task(mut foc_pwm: FocPwm<RightDriverImpl<'static>>) {
  rprintln!("Left motor  only used for angle sensor measurement");
  // left is running in full foc mode, right is running sensorless
  let mut foc = FocSimple::new(7);
  let mut ticker = Ticker::every(Duration::from_millis(1));

  let parm = FocParam::new_fp(2.0, 0.00, 0.1);
  _ = foc.set_foc_mode(foc_simple::foc::EFocMode::Velocity(parm));
  foc.set_speed(I16F16::TAU * 5);
  loop {
    // 2. Run the FOC algorithm
    match foc.update(EFocAngle::SensorLess) {
      Err(_) => {
        rprintln!("Error while updating foc");
      }
      Ok((angle, torque)) => foc_pwm.update(angle,torque, None).unwrap(),
    }
    let now = Instant::now().as_millis() as usize;
    foc.update_velocity(now);
    ticker.next().await;
  }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  rtt_init_print!();

  let p = embassy_stm32::init(Default::default());
  rprintln!("Start test foc velociy test");

  let led0 = Output::new(p.PB12, Level::High, Speed::Medium);

  let _led1 = Output::new(p.PB13, Level::High, Speed::Medium);

  let a0 = PwmPin::new_ch2(p.PA7, OutputType::PushPull);
  let b0 = PwmPin::new_ch3(p.PB0, OutputType::PushPull);
  let c0 = PwmPin::new_ch4(p.PB1, OutputType::PushPull);

  let a2 = PwmPin::new_ch4(p.PB9, OutputType::PushPull);
  let b2 = PwmPin::new_ch2(p.PA1, OutputType::PushPull);
  let c2 = PwmPin::new_ch3(p.PB8, OutputType::PushPull);

  let pwm3 = SimplePwm::<'static>::new(p.TIM3, None, Some(a0), Some(b0), Some(c0), Hertz(25000), CountingMode::CenterAlignedUpInterrupts);
  let pwm2 = SimplePwm::<'static>::new(p.TIM2, None, Some(b2), None, None, Hertz(25000), CountingMode::CenterAlignedUpInterrupts);
  let pwm4 = SimplePwm::<'static>::new(p.TIM4, None, None, Some(c2), Some(a2), Hertz(25000), CountingMode::CenterAlignedUpInterrupts);

  let channels2 = pwm2.split();
  let channels3 = pwm3.split();
  let channels4 = pwm4.split();

  rprintln!("Max duty: {:?}", channels3.ch1.max_duty_cycle());

  let left_driver = LeftDriverImpl::new(channels3.ch2, channels3.ch3, channels3.ch4);
  let right_driver = RightDriverImpl::new(channels2.ch2, channels4.ch4, channels4.ch3);

  let left_pwm = FocPwm::new(left_driver, EModulation::Sinusoidal, 160, 0.5);
  let right_pwm = FocPwm::new(right_driver, EModulation::SpaceVector, 160, 0.5);

  _ = spawner.spawn(left_motor_task(left_pwm));
  _ = spawner.spawn(right_motor_task(right_pwm));
  _ = spawner.spawn(ticker_task(led0));
  rprintln!("All tasks spawned");

  loop {
    Timer::after(Duration::from_millis(1000)).await;
  }
}
