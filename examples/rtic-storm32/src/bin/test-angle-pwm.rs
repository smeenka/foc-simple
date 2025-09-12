#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprint, rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM1;
use stm32f1xx_hal::timer::PwmInput;
use stm32f1xx_hal::timer::Tim1NoRemap;

/// PWM input monitor type
pub(crate) type PwmMonitor = PwmInput<TIM1, Tim1NoRemap, (Pin<'A', 8>, Pin<'A', 9>)>;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use rtic_monotonics::fugit::{Rate, RateExtU32};
    use stm32f1xx_hal::{
        afio::AfioExt,
        flash::FlashExt,
        gpio::{
            Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB12, PC13,
        },
        pac::Peripherals,
        rcc::{Clocks, RccExt},
        time::KiloHertz,
        timer::{Configuration, ReadMode, Tim2NoRemap, Timer},
    };
    use fixed::types::{I16F16,I8F24};

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PB12<Output<PushPull>>,
        state: bool,
        monitor: PwmMonitor,
        clocks: Clocks,
        counter: usize,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Initialize the systick interrupt & obtain the token to prove that we did
        Mono::start(cx.core.SYST, 36_000_000);

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let channels = rtt_init_default!();
        rtt_target::set_print_channel(channels.up.0);
        rprintln!("Test as5600 PWM Output");

        // Setup LED
        let mut gpiob = cx.device.GPIOB.split();
        let led = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

        // Configure TIM1 into PWM input mode.
        // The period time MUST be below 32768, otherwise the I16F16 will overflow
        // Note: as a side-effect TIM1's interrupt is enabled and fires whenever a capture-compare
        //      cycle is complete. See the reference manual's paragraphs on PWM Input.

        let mut dbg = cx.device.DBGMCU;
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        // disable jtag
        //t (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let monitor = Timer::new(cx.device.TIM1, &clocks).pwm_input(
            (gpioa.pa8, gpioa.pa9),
            &mut afio.mapr,
            &mut dbg,
            Configuration::RawFrequency(450u32.Hz()),
        );
        // Schedule the blinking task
        blink::spawn().ok();

        (
            Shared {},
            Local {
                led,
                state: false,
                monitor,
                clocks,
                counter:0
            },
        )
    }

    #[task(local = [led, state])]
    async fn blink(cx: blink::Context) {
        loop {
            //let count = COUNTER.swap(0, Ordering::SeqCst);
            let count = 0;
            rprintln!("blink. Count:{}", count);
            if *cx.local.state {
                cx.local.led.set_high();
                *cx.local.state = false;
            } else {
                cx.local.led.set_low();
                *cx.local.state = true;
            }
            Mono::delay(1000.millis()).await;
        }
    }


    #[task(binds=TIM1_CC, local=[monitor,clocks, counter])]
    fn tim2_cc(mut cx: tim2_cc::Context) {
        let monitor: &PwmMonitor = cx.local.monitor;
        let mut counter = cx.local.counter;

        // observe duty cycle
        let (duty, period) = monitor.read_duty(ReadMode::Instant).unwrap(); 

        // how many ticks we have for per duty cycle unit. In total we have 4096 + 128 + 128 duty cycle units
        let tpu_f32:f32 = period as f32 / 4352.0; 
        let ticks_per_unit:I16F16 = I16F16::from_num(tpu_f32);
        // how many duty cycle units is the period
        let duty_ticks = I16F16::from_num(duty) / ticks_per_unit;
        // substract the start period ticks
        let duty_ticks = duty_ticks - I16F16::from_num(128);
        // angle is actual duty cycle fraction times TAU
        let angle = I16F16::TAU * duty_ticks/I16F16::from_num(4096);

        *counter += 1;
        if *counter > 100 {
          *counter = 0;
          if period > 32767 {
            rprintln!{"Period too high, lower the pwm frequency!"}
          } 
          rprintln!("period:{} duty:{}  angle:{} tpu:{}", period, duty, angle, ticks_per_unit);
        } 
        // leaving critical section
    }
}
