//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::foc_simple::FocSimple;
use foc_simple::foc::foc_embassy::{FocEmbassy, FocSimpleCommand};
use foc_simple::foc::{EFocMode, EModulation};
use foc_simple::EFocAngleSensor;
use foc_simple::EFocCommand;
use foc_simple::FocParam;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};
use storm32::foc_impl::{LeftDriverImpl, RightDriverImpl, SensorAngleImpl, SensorCurrentImpl};

bind_interrupts!(
    struct Irqs {}
);

#[embassy_executor::task]
async fn motor_r_task(
    mut foc_async: FocEmbassy<SensorAngleImpl, SensorCurrentImpl, RightDriverImpl<'static>>,
) {
    foc_async.task().await;
}
#[embassy_executor::task]
async fn motor_l_task(
    mut foc_async: FocEmbassy<SensorAngleImpl, SensorCurrentImpl, LeftDriverImpl<'static>>,
) {
    foc_async.task().await;
}

#[embassy_executor::task]
pub async fn ticker_task(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(300));
    loop {
        led.toggle();
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

    let i2c1 = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(200_000), Default::default());
    //let i2c2 = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(200_000), Default::default());

    // wrap as5600 object to make the SensorAngle interface available
    //let sensor_angle1 = SensorAngleImpl::from_as5600(i2c2);
    let sensor_angle2 = SensorAngleImpl::from_as5600(i2c1);

    // all objects for the left side (motor nr 0)
    let left_driver = LeftDriverImpl::new(channels3.ch2, channels3.ch3, channels3.ch4);
    let foc_pwm_l = FocPwm::new(left_driver, EModulation::Sinusoidal, 160, 0.5);
    let foc_l = FocSimple::new(7);
    let foc_async_l = FocEmbassy::<SensorAngleImpl, SensorCurrentImpl, LeftDriverImpl>::new(
        0,
        foc_l,
        foc_pwm_l,
        Ticker::every(Duration::from_millis(1)),
        Ticker::every(Duration::from_millis(10)),
    );
    _ = spawner.spawn(motor_l_task(foc_async_l));

    // all objects for the right side (motor nr 1)
    let right_driver = RightDriverImpl::new(channels2.ch2, channels4.ch4, channels4.ch3);
    let foc_pwm_r = FocPwm::new(right_driver, EModulation::SpaceVector, 160, 0.5);
    let foc_r = FocSimple::new(7);
    // create the async wrapper
    let mut foc_async_r = FocEmbassy::<SensorAngleImpl, SensorCurrentImpl, RightDriverImpl>::new(
        1,
        foc_r,
        foc_pwm_r,
        Ticker::every(Duration::from_millis(1)),
        Ticker::every(Duration::from_millis(10)),
    );
    foc_async_r.set_angle_sensor(EFocAngleSensor::ShaftSensor(sensor_angle2));
    _ = spawner.spawn(motor_r_task(foc_async_r));
    _ = spawner.spawn(ticker_task(led0));

    FocSimpleCommand::send_command(1, EFocCommand::FocMode(EFocMode::Calibration(None)));
    Timer::after(Duration::from_secs(10)).await;
    let parm = FocParam::new_fp(0.2, 0.01, 0.05);
    let mode = EFocMode::Velocity(parm);
    FocSimpleCommand::send_command(1, EFocCommand::FocMode(mode));
    FocSimpleCommand::send_command(1, EFocCommand::Speed(I16F16::TAU));
    FocSimpleCommand::send_command(0, EFocCommand::FocMode(mode));
    FocSimpleCommand::send_command(0, EFocCommand::Speed(I16F16::TAU));

    rprintln!("All tasks spawned");
    loop {
        Timer::after(Duration::from_secs(1)).await;
    } 
}  
