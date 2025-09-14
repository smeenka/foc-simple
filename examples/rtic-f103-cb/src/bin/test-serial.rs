#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use core::fmt::Write;
use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
  use rtic_monotonics::fugit::{Rate, RateExtU32};
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::{Peripherals, USART2},
    rcc::{self, Clocks, RccExt},
    serial::{Config, Rx, Serial, Tx},
    time::{Bps, KiloHertz},
    timer::{Tim2NoRemap, Timer},
  };

  use super::*;

  /* bring dependencies into scope */

  #[shared]
  struct Shared {}

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    button: PC13<Input<PullUp>>,
    tx: Tx<USART2>,
    rx: Rx<USART2>,
  }

  #[init]
  fn init(mut cx: init::Context) -> (Shared, Local) {
    // Setup clocks
    let mut flash = cx.device.FLASH.constrain();
    let mut rcc = cx
      .device
      .RCC
      .freeze(rcc::Config::hsi().sysclk(72.MHz()).pclk1(36.MHz()), &mut flash.acr);
    let mut afio = cx.device.AFIO.constrain(&mut rcc);

    // Initialize the systick interrupt & obtain the token to prove that we did
    Mono::start(cx.core.SYST, 36_000_000);

    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Test Serial serial2 interface over usb");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    // Setup LED
    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    //t (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut button = gpioc.pc13.into_pull_up_input(&mut gpioc.crh);
    button.make_interrupt_source(&mut afio);
    button.enable_interrupt(&mut cx.device.EXTI);
    button.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

    // USART2
    let pin_tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let pin_rx = gpioa.pa3;

    // Set up the usart device.
    let serial = Serial::new(
      cx.device.USART2,
      (pin_tx, pin_rx),
      Config::default().baudrate(Bps(115_200)),
      &mut rcc,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, rx) = serial.split();

    let number = 103;
    writeln!(tx, "Hello formatted string {}", number).unwrap();

    // Schedule the blinking task
    blink::spawn().ok();
    serial::spawn().ok();
    (Shared {}, Local { led, button, tx, rx })
  }

  #[task(local = [led])]
  async fn blink(cx: blink::Context) {
    loop {
      cx.local.led.toggle();
      Mono::delay(1000.millis()).await;
    }
  }

  #[task(local = [tx, rx])]
  async fn serial(cx: serial::Context) {
    let mut count = 0;
    loop {
      for _x in 0..10 {
        core::writeln!(
          cx.local.tx,
          " Please send characters to the device. Count nr:{:02}\r\n",
          count
        )
        .unwrap();
        count += 1;
        let mut buffer = [0_u8; 64];
        match cx.local.rx.read() {
          Ok(a) => {
            cx.local.tx.write_u8(65);
            cx.local.tx.write_u8(a);
            cx.local.tx.flush();
          }
          Err(_) => (),
        }
      }
      Mono::delay(1000.millis()).await;
    }
  }

  /// note that  C13 is connected to EXTI3
  #[task(binds=EXTI15_10,  local=[button])]
  fn button_pressed(mut context: button_pressed::Context) {
    context.local.button.clear_interrupt_pending_bit();
    rprintln!("EXTI15_10 Button on C13  pressed");
  }
}
