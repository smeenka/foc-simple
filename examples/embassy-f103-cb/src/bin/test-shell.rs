//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_f103_cb::foc_impl::hall_sensor_impl::HallSensorImpl;
use embassy_f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
use embassy_f103_cb::foc_impl::{FocSerialImpl, SensorAngleDummy, SensorCurrentImpl};
use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Ticker, Timer};
use foc_simple::foc::foc_embassy::FocEmbassy;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::foc_simple::FocSimple;
use foc_simple::foc::EModulation;
use foc_simple::EFocAngleSensor;
use foc_simple::FocCommand;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};

bind_interrupts!(
    struct Irqs {
          USART2 => usart::InterruptHandler<peripherals::USART2>;

    }
);

#[embassy_executor::task]
async fn motor_task(mut foc_async: FocEmbassy<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl<'static>>) {
  foc_async.task().await;
}

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
pub async fn command_task(mut command: FocCommand<FocSerialImpl>) {
  command.task().await;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  let channels = rtt_init_default!();
  rtt_target::set_print_channel(channels.up.0);

  let p = embassy_stm32::init(Default::default());

  let led1 = Output::new(p.PB2, Level::High, Speed::Medium);

  let uart = Uart::new(p.USART2, p.PA3, p.PA2, Irqs, p.DMA1_CH7, p.DMA1_CH6, Config::default()).unwrap();

  Timer::after(Duration::from_millis(100)).await;
  rprintln!("Start shell for nucleo");

  let a0 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
  let b0 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
  let c0 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

  let ea0 = Output::new(p.PC10, Level::High, Speed::Medium);
  let eb0 = Output::new(p.PC11, Level::High, Speed::Medium);
  let ec0 = Output::new(p.PC12, Level::High, Speed::Medium);

  let pwm1 = SimplePwm::<'static>::new(p.TIM1, Some(a0), Some(b0), Some(c0), None, Hertz(25000), CountingMode::CenterAlignedUpInterrupts);

  let mut pwm_driver = PwmDriverImpl::new(pwm1, ea0, eb0, ec0);
  pwm_driver.enable();

  // PB3 has problems, is not routed to the CPU for whatever reason
  // PB5 is connected to PB3 with a jumper
  let _h2_orig = Input::new(p.PB3, Pull::None);

  let h1 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);
  let h2 = ExtiInput::new(p.PB14, p.EXTI14, Pull::Down);
  let h3 = ExtiInput::new(p.PB10, p.EXTI10, Pull::Down);
  let sensor = HallSensorImpl::new(0, h3, h2, h1, 3);
  _ = spawner.spawn(hall_sensor_task(sensor));

  let foc_pwm = FocPwm::new(pwm_driver, EModulation::Sinusoidal, 160, 0.4);

  // create the foc for the right driver
  let foc = FocSimple::new(3);

  // create the async wrapper
  let mut foc_async = FocEmbassy::<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl>::new(0, foc, foc_pwm, Ticker::every(Duration::from_millis(1)), Ticker::every(Duration::from_millis(10)));
  foc_async.set_angle_sensor(EFocAngleSensor::HallSensor);
  _ = spawner.spawn(motor_task(foc_async));

  let parser = FocSerialImpl::new(uart);
  let command = FocCommand::new(parser);

  _ = spawner.spawn(command_task(command));
  _ = spawner.spawn(ticker1_task(led1));

  rprintln!("All tasks spawned");
  loop {
    Timer::after(Duration::from_secs(1)).await;
  }
}
