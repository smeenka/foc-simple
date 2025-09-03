#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::timer::PwmInput;
use stm32f1xx_hal::timer::Tim2NoRemap;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use super::*;
  use fixed::types::I16F16;
  use foc_simple::foc::foc_hall_sensor::FocHallSensor;
  use foc_simple::foc::foc_rtic::FocRtic;
  use foc_simple::foc::foc_rtic::FocSimpleCommand;
  use foc_simple::foc::foc_rtic::HALL_ANGLES;
  use foc_simple::foc::foc_rtic::SIGNAL_10MS;
  use foc_simple::foc::foc_rtic::SIGNAL_1MS;
  use foc_simple::foc::foc_simple::FocSimple;
  use foc_simple::foc::EFocMode;
  use foc_simple::foc::{foc_pwm::FocPwm, EModulation};
  use foc_simple::{EFocAngle, FocParam};
  use rtic_f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
  use rtic_f103_cb::foc_impl::SensorAngleDummy;
  use rtic_f103_cb::foc_impl::SensorCurrentImpl;
  use rtic_monotonics::fugit::{Instant, Rate, RateExtU32};
  use stm32f1xx_hal::gpio::{PA15, PB10, PB14};
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::Peripherals,
    rcc::{Clocks, RccExt},
    time::KiloHertz,
    timer::{Channel, Configuration, PwmExt, ReadMode, Tim1NoRemap, Tim2NoRemap, Timer},
  };
  /* bring dependencies into scope */

  #[shared]
  struct Shared {
    position: u16,
  }

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    clocks: Clocks,
    foc_rtic: FocRtic<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl>,
    hall1: PA15<Input<PullUp>>,
    hall2: PB14<Input<PullUp>>,
    hall3: PB10<Input<PullUp>>,
    hall_sensor: FocHallSensor,
  }

  #[init]
  fn init(mut cx: init::Context) -> (Shared, Local) {
    // Setup clocks
    let mut flash = cx.device.FLASH.constrain();
    let rcc = cx.device.RCC.constrain();

    let mut clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()).freeze(&mut flash.acr);

    // Initialize Mono interrupt & clocks
    Mono::start(cx.core.SYST, 36_000_000);

    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("init");

    let mut gpioa = cx.device.GPIOA.split();
    let mut gpiob = cx.device.GPIOB.split();
    let mut gpioc = cx.device.GPIOC.split();
    let mut afio = cx.device.AFIO.constrain();

    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut hall1 = pa15.into_pull_up_input(&mut gpioa.crh);
    hall1.make_interrupt_source(&mut afio);
    hall1.enable_interrupt(&mut cx.device.EXTI);
    hall1.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    let mut hall2 = gpiob.pb14.into_pull_up_input(&mut gpiob.crh);
    hall2.make_interrupt_source(&mut afio);
    hall2.enable_interrupt(&mut cx.device.EXTI);
    hall2.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    let mut hall3 = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
    hall3.make_interrupt_source(&mut afio);
    hall3.enable_interrupt(&mut cx.device.EXTI);
    hall3.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    let hall_sensor = FocHallSensor::new(0, 3);
    // pwm channels
    let a0 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    let b0 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let c0 = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
    let pins = (a0, b0, c0);

    // channel enable
    let ea0 = gpioc.pc10.into_push_pull_output(&mut gpioc.crh);
    let eb0 = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);
    let ec0 = gpioc.pc12.into_push_pull_output(&mut gpioc.crh);

    let mut pwm = cx.device.TIM1.pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 25.kHz(), &mut clocks);
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    let channels = pwm.split();
    // Enable clock on each of the channels
    let max = channels.0.get_max_duty();
    rprintln!("Max duty:{}", max);

    let mut pwm_driver = PwmDriverImpl::new(channels.0, channels.1, channels.2, ea0.erase(), eb0.erase(), ec0.erase());
    pwm_driver.enable();
    let mut foc_pwm = FocPwm::new(pwm_driver, EModulation::Sinusoidal, max, 0.4);
    let foc_simple = FocSimple::new(3);
    let foc_rtic = FocRtic::<SensorAngleDummy, SensorCurrentImpl, PwmDriverImpl>::new(0, foc_simple, foc_pwm);
    // Schedule the blinking task
    blink_task::spawn().ok();
    motor_task::spawn().ok();
    ticker_task::spawn().ok();

    (
      Shared { position: 0 },
      Local {
        led,
        clocks,
        foc_rtic,
        hall1,
        hall2,
        hall3,
        hall_sensor,
      },
    )
  }

  #[task(local = [led], priority = 1)]
  async fn blink_task(cx: blink_task::Context) {
    let mut instant = Mono::now();
    loop {
      cx.local.led.toggle();
      instant += 1.secs();
      Mono::delay_until(instant).await;
    }
  }
  // ticker task generating signals with 1000 hrz and 100 hrz
  #[task(priority = 2)]
  async fn ticker_task(cx: ticker_task::Context) {
    rprintln!("Starting ticker task");
    let mut instant = Mono::now();
    let mut counter = 0;
    loop {
      // 2. Run the FOC algorithm
      SIGNAL_1MS[0].signal(());
      if counter % 10 == 0 {
        let now = (Mono::now().ticks());
        // this signal drives the velocity calculations and filters
        SIGNAL_10MS[0].signal(now as usize);
      }
      counter += 1;
      instant += 1.millis();
      Mono::delay_until(instant).await;
    }
  }

  // motor task, waiting for incoming signals for 1 ms, 10 ms, hall sensor and commands
  #[task(local = [foc_rtic], priority = 2)]
  async fn motor_task(cx: motor_task::Context) {
    rprintln!("BLCD motor  with sinusiode driver, 1/4 torque, 1 rad/s");
    let mut foc_rtic = cx.local.foc_rtic;
    //foc.set_foc_mode(EFocMode::Calibration(None)).unwrap();
    let param = FocParam::new_fp(0.05, 0.001, 0.0);
    foc_rtic.get_foc().set_foc_mode(foc_simple::foc::EFocMode::Velocity(param)).unwrap();
    foc_rtic.get_foc().set_speed(I16F16::TAU);
    foc_rtic.get_foc().set_acceleration(I16F16::ZERO);
    foc_rtic.get_foc().set_torque(I16F16::ONE / 4); // current torque, hall sensor and commands
    foc_rtic.task().await;
  }

  //  hall sensor
  #[task(binds=EXTI15_10,  local=[hall1, hall2, hall3, hall_sensor], priority = 3)]
  fn hall_sensor(mut context: hall_sensor::Context) {
    let h1 = context.local.hall1;
    let h2 = context.local.hall2;
    let h3 = context.local.hall3;
    h1.clear_interrupt_pending_bit();
    h2.clear_interrupt_pending_bit();
    h3.clear_interrupt_pending_bit();
    let mut phase = 0;
    if h1.is_high() {
      phase |= 1;
    }
    if h2.is_high() {
      phase |= 2;
    }
    if h3.is_high() {
      phase |= 4;
    }
    match context.local.hall_sensor.update(phase) {
      Err(_) => rprintln!("Error while fetching hall angle"),
      Ok(angle) => {
        HALL_ANGLES[0].signal(angle);
      }
    }
  }
}
