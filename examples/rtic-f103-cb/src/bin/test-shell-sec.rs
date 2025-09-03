#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use core::fmt::Write;
use foc_simple::FocCommand;
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
    use rtic_f103_cb::foc_impl::FocSerialImpl;
    use rtic_monotonics::fugit::{Rate, RateExtU32};
    use stm32f1xx_hal::{
        afio::AfioExt,
        flash::FlashExt,
        gpio::{
            Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13,
        },
        pac::{Peripherals, USART2},
        rcc::{Clocks, RccExt},
        serial::{Config, Rx, Serial, Tx},
        time::{Bps, KiloHertz},
        timer::{Configuration, ReadMode, Tim2NoRemap, Timer},
    };

    use super::*;

    /* bring dependencies into scope */

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PB2<Output<PushPull>>,
        button: PC13<Input<PullUp>>,
        command: FocCommand<FocSerialImpl>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Initialize the systick interrupt & obtain the token to prove that we did
        Mono::start(cx.core.SYST, 36_000_000);

        let mut clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let channels = rtt_init_default!();
        rtt_target::set_print_channel(channels.up.0);
        rprintln!("Test Serial serial2 interface over usb");

        // Setup LED
        let mut gpiob = cx.device.GPIOB.split();
        let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        // disable jtag
        //t (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut gpioc = cx.device.GPIOC.split();
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
            &mut afio.mapr,
            Config::default().baudrate(Bps(115_200)),
            &mut clocks,
        );
        // Split the serial struct into a receiving and a transmitting part
        let (tx, rx) = serial.split();

        let parser = FocSerialImpl::new(tx, rx);
        let command = FocCommand::new(parser);

        // Schedule the blinking task
        blink::spawn().ok();
        command::spawn().ok();
        (
            Shared {},
            Local {
                led,
                button,
                command,
            },
        )
    }

    #[task(local = [led], priority = 2)]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(1000.millis()).await;
        }
    }


    #[task(local = [command], priority = 1)]
    async fn command(context: command::Context) {
        context.local.command.task().await;
    }

    #[task(binds=EXTI15_10,  local=[button] , priority = 2)]
    fn button_pressed(mut context: button_pressed::Context) {
        context.local.button.clear_interrupt_pending_bit();
        rprintln!("EXTI15_10 Button on C13  pressed");
    }
}
