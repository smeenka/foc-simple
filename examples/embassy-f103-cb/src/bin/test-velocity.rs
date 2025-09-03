//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::foc_simple::FocSimple;
use foc_simple::foc::{EFocMode, EModulation};
use foc_simple::FocParam;
use embassy_f103_cb::foc_impl::hall_sensor_impl::HallSensorImpl;
use embassy_f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
use foc_simple::EFocAngleSensor;
use foc_simple::foc::foc_embassy::FocEmbassy;
use foc_simple::foc::foc_embassy::FocSimpleCommand;
use foc_simple::EFocCommand;
use embassy_f103_cb::foc_impl::{SensorAngleDummy, SensorCurrentImpl};
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};

bind_interrupts!(
    struct Irqs {}
);

#[embassy_executor::task]
pub async fn ticker_task(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        led.toggle();
        ticker.next().await;
    }
}



#[embassy_executor::task]
pub async fn ticker1_task(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(300));
    loop {
        led.toggle();
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn hall_sensor_task(mut sensor: HallSensorImpl) {
    let mut ticker = Ticker::every(Duration::from_secs(10));
    let mut errors = 0;
    loop {
        match select(sensor.update(), ticker.next()).await {
            Either::First(_) => (),
            Either::Second(_) => {
                let ec = sensor.get_errorcount();
                if ec != errors {
                    rprintln!("Errors:{}  ", errors);
                    errors = ec;
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn motor_task(mut foc: FocEmbassy::<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl<'static>>) {
    rprintln!("BLCD motor  with sinusiode driver, 1/4 torque, 1 rad/s");
    foc.task().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p = embassy_stm32::init(Default::default());
    rprintln!("Start test foc velociy test");

    let led1 = Output::new(p.PB2, Level::High, Speed::Medium);

    let a0 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let b0 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let c0 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

    let ea0 = Output::new(p.PC10, Level::High, Speed::Medium);
    let eb0 = Output::new(p.PC11, Level::High, Speed::Medium);
    let ec0 = Output::new(p.PC12, Level::High, Speed::Medium);

    let pwm1 = SimplePwm::<'static>::new(
        p.TIM1,
        Some(a0),
        Some(b0),
        Some(c0),
        None,
        Hertz(25000),
        CountingMode::CenterAlignedUpInterrupts,
    );

    let mut pwm_driver = PwmDriverImpl::new(pwm1, ea0, eb0, ec0);
    pwm_driver.enable();

    // PB3 has problems, is not routed to the CPU for whatever reason
    let _h2_orig = Input::new(p.PB3, Pull::None);

    let h1 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);
    let h2 = ExtiInput::new(p.PB14, p.EXTI14, Pull::Down);
    let h3 = ExtiInput::new(p.PB10, p.EXTI10, Pull::Down);
    let sensor = HallSensorImpl::new(0, h1, h2, h3, 3);

    let foc_pwm = FocPwm::new(pwm_driver, EModulation::Sinusoidal, 160, 0.3);

    // create the foc for the right driver
    let foc = FocSimple::new(3);
    // create the embasy async wrapper
    // create the async wrapper
    let mut foc_async = FocEmbassy::<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl>::new(
        0,
        foc,
        foc_pwm,
        Ticker::every(Duration::from_millis(10)),
        Ticker::every(Duration::from_millis(10)),
    );
    foc_async.set_angle_sensor(EFocAngleSensor::HallSensor);
    _ = spawner.spawn(ticker1_task(led1));
    _ = spawner.spawn(hall_sensor_task(sensor));
    _ = spawner.spawn(motor_task(foc_async));
    rprintln!("All tasks spawned");

    FocSimpleCommand::send_command(0, EFocCommand::FocMode(EFocMode::Calibration(None)));
    Timer::after(Duration::from_secs(10)).await;
    let parm = FocParam::new_fp(0.1, 0.00, 0.01);
    let mode = EFocMode::Velocity(parm);
    FocSimpleCommand::send_command(0, EFocCommand::FocMode(mode));
    FocSimpleCommand::send_command(0, EFocCommand::Speed(I16F16::TAU));


    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
