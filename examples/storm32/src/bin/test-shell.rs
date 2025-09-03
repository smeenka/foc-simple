//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Ticker, Timer};
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::foc_simple::FocSimple;
use foc_simple::foc::foc_embassy::{FocEmbassy};
use foc_simple::foc::{EModulation};
use foc_simple::FocCommand;
use foc_simple::EFocAngleSensor;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};
use storm32::foc_impl::{
    FocSerialImpl, LeftDriverImpl, RightDriverImpl, SensorAngleImpl, SensorCurrentImpl,
};

bind_interrupts!(
    struct Irqs {
          USART1 => usart::InterruptHandler<peripherals::USART1>;

    }
);

#[embassy_executor::task]
async fn left_task(
    mut foc_async: FocEmbassy<SensorAngleImpl, SensorCurrentImpl, LeftDriverImpl<'static>>,
) {
    foc_async.task().await;
}

#[embassy_executor::task]
async fn right_task(
    mut foc_async: FocEmbassy<SensorAngleImpl, SensorCurrentImpl, RightDriverImpl<'static>>,
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

#[embassy_executor::task]
pub async fn command_task(mut command: FocCommand<FocSerialImpl>) {
    command.task().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p = embassy_stm32::init(Default::default());

    let led0 = Output::new(p.PB12, Level::High, Speed::Medium);

    let _led1 = Output::new(p.PB13, Level::High, Speed::Medium);

    let mut config = Config::default();
    config.baudrate = 115200;

    let uart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Config::default(),
    )
    .unwrap();
    Timer::after(Duration::from_millis(100)).await;
    rprintln!("Start test foc velociy test");

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

    Timer::after(Duration::from_millis(100)).await;
    rprintln!("Max duty: {:?}", channels3.ch1.max_duty_cycle());

    let i2c1 = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    //let i2c2 = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(200_000), Default::default());
    Timer::after(Duration::from_millis(100)).await;

    // wrap as5600 object to make the SensorAngle interface available
    let sensor_angle1 = SensorAngleImpl::from_as5600(i2c1);
    //let sensor_angle2 = SensorAngleImpl::from_as5600(i2c2);
    // crate the foc_angle object
    Timer::after(Duration::from_millis(100)).await;

    let pwm_driver_l = LeftDriverImpl::new(channels3.ch2, channels3.ch3, channels3.ch4);
    let pwm_driver_r = RightDriverImpl::new(channels2.ch2, channels4.ch4, channels4.ch3);

    let foc_pwm_l = FocPwm::new(pwm_driver_l, EModulation::Sinusoidal, 160, 0.5);
    let foc_pwm_r = FocPwm::new(pwm_driver_r, EModulation::SpaceVector, 160, 0.5);

    // create the foc for the right driver
    let foc = FocSimple::new(7);
    // create the async wrapper
    let mut foc_async = FocEmbassy::<SensorAngleImpl, SensorCurrentImpl, RightDriverImpl>::new(
        0,
        foc,
        foc_pwm_r,
        Ticker::every(Duration::from_millis(1)),
        Ticker::every(Duration::from_millis(10)),
    );
    foc_async.set_angle_sensor(EFocAngleSensor::ShaftSensor(sensor_angle1));
    _ = spawner.spawn(right_task(foc_async));

    // create the sync foc for the left driver
    let foc = FocSimple::new(7);
    //create the async wrapper
    let mut foc_async = FocEmbassy::<SensorAngleImpl, SensorCurrentImpl, LeftDriverImpl>::new(
        1,
        foc,
        foc_pwm_l,
        Ticker::every(Duration::from_millis(1)),
        Ticker::every(Duration::from_millis(10)),
    );
    foc_async.set_angle_sensor(EFocAngleSensor::SensorLess);
    _ = spawner.spawn(left_task(foc_async));

    let parser = FocSerialImpl::new(uart);
    let command = FocCommand::new(parser);

    _ = spawner.spawn(command_task(command));
    _ = spawner.spawn(ticker_task(led0));

    rprintln!("All tasks spawned");
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
