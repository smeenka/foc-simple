//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]


use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::EModulation;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};
use storm32::foc_impl::{LeftDriverImpl, RightDriverImpl};

// --- Motor and Control Parameters ---
#[embassy_executor::task]
pub async fn ticker_task(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        led.toggle();
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn left_motor_task(mut foc_pwm: FocPwm<LeftDriverImpl<'static>>) {
    rprintln!("Left motor  with sinusoidal driver, 1/4 torque, one turn per  7 seconds");
    let mut curr_angle = I16F16::ZERO;
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        curr_angle += 6 * I16F16::ONE / 1000;

        // 2. Run the FOC algorithm
        foc_pwm
            .update(curr_angle, I16F16::ONE / 4, None)
            .unwrap();
        ticker.next().await;
    }
}
#[embassy_executor::task]
pub async fn right_motor_task(mut foc_pwm: FocPwm<RightDriverImpl<'static>>) {
    rprintln!("Right motor  with trapeziodal driver, 1/4 torque, 10 turns per second");
    let mut curr_angle = I16F16::ZERO;
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        curr_angle += 6 * I16F16::ONE / 20;

        // 2. Run the FOC algorithm
        foc_pwm
            .update(curr_angle, I16F16::ONE / 2, None)
            .unwrap();
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p = embassy_stm32::init(Default::default());
    rprintln!("Start test pwm drivers");

    let a0 = PwmPin::new_ch2(p.PA7, OutputType::PushPull);
    let b0 = PwmPin::new_ch3(p.PB0, OutputType::PushPull);
    let c0 = PwmPin::new_ch4(p.PB1, OutputType::PushPull);

    let a2 = PwmPin::new_ch4(p.PB9, OutputType::PushPull);
    let b2 = PwmPin::new_ch2(p.PA1, OutputType::PushPull);
    let c2 = PwmPin::new_ch3(p.PB8, OutputType::PushPull);

    let led0 = Output::new(p.PB12, Level::High, Speed::Medium);
    let _led1 = Output::new(p.PB13, Level::High, Speed::Medium);

    let pwm3 = SimplePwm::<'static>::new(
        p.TIM3,
        None,
        Some(a0),
        Some(b0),
        Some(c0),
        Hertz(25000),
        CountingMode::CenterAlignedUpInterrupts,
    );
    let pwm2 = SimplePwm::<'static>::new(
        p.TIM2,
        None,
        Some(b2),
        None,
        None,
        Hertz(25000),
        CountingMode::CenterAlignedUpInterrupts,
    );
    let pwm4 = SimplePwm::<'static>::new(
        p.TIM4,
        None,
        None,
        Some(c2),
        Some(a2),
        Hertz(25000),
        CountingMode::CenterAlignedUpInterrupts,
    );

    let channels2 = pwm2.split();
    let channels3 = pwm3.split();
    let channels4 = pwm4.split();

    rprintln!("Max duty: {:?}", channels3.ch1.max_duty_cycle());

    let left_driver = LeftDriverImpl::new(channels3.ch2, channels3.ch3, channels3.ch4);
    let right_driver = RightDriverImpl::new(channels2.ch2, channels4.ch3, channels4.ch4);

    let left_pwm = FocPwm::new(left_driver, EModulation::Sinusoidal, 160, 0.5);
    let right_pwm = FocPwm::new(right_driver, EModulation::SpaceVector, 160, 0.5);
    _ = spawner.spawn(left_motor_task(left_pwm));
    _ = spawner.spawn(right_motor_task(right_pwm));
    _ = spawner.spawn(ticker_task(led0));
    rprintln!("All tasks spawned");

    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
