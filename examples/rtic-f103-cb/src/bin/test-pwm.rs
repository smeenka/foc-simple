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

/// PWM input monitor type
pub(crate) type PwmMonitor = PwmInput<TIM2, Tim2NoRemap, (Pin<'A', 0>, Pin<'A', 1>)>;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use fixed::types::I16F16;
    use foc_simple::foc::{foc_pwm::FocPwm, EModulation};
    use rtic_f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
    use rtic_monotonics::fugit::{Rate, RateExtU32};
    use stm32f1xx_hal::{
        afio::AfioExt,
        flash::FlashExt,
        gpio::{
            Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13,
        },
        pac::Peripherals,
        rcc::{Clocks, RccExt},
        time::KiloHertz,
        timer::{Channel, Configuration, PwmExt, ReadMode, Tim1NoRemap, Tim2NoRemap, Timer},
    };

    use super::*;

    /* bring dependencies into scope */

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PB2<Output<PushPull>>,
        clocks: Clocks,
        foc_pwm: FocPwm<PwmDriverImpl>,
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
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let channels = rtt_init_default!();
        rtt_target::set_print_channel(channels.up.0);
        rprintln!("Test foc_pwm module");

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();
        let mut afio = cx.device.AFIO.constrain();

        // onboard led of the blcd driver
        let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);


        // pwm channels
        let a0 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        let b0 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let c0 = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
        let pins = (a0, b0, c0);

        // channel enable
        let ea0 = gpioc.pc10.into_push_pull_output(&mut gpioc.crh);
        let eb0 = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);
        let ec0 = gpioc.pc12.into_push_pull_output(&mut gpioc.crh);

        // disable jtag
        //t (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Configure TIM1 into PWM output mode.
        let mut pwm =
            cx.device
                .TIM1
                .pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 25.kHz(), &mut clocks);
        pwm.enable(Channel::C1);
        pwm.enable(Channel::C2);
        pwm.enable(Channel::C3);
        let channels = pwm.split();
        // Figure out the max duty cycle of the channels
        let max = channels.0.get_max_duty();
        rprintln!("Max duty:{}", max);

        let mut pwm_driver = PwmDriverImpl::new(
            channels.0,
            channels.1,
            channels.2,
            ea0.erase(),
            eb0.erase(),
            ec0.erase(),
        );
        pwm_driver.enable();
        let mut foc_pwm = FocPwm::new(pwm_driver, EModulation::Sinusoidal, max, 0.4);

        // Schedule the blinking task
        blink_task::spawn().ok();
        motor_task::spawn().ok();

        (
            Shared { },
            Local {
                led,
                clocks,
                foc_pwm,
            },
        )
    }

    #[task(local = [led])]
    async fn blink_task(cx: blink_task::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(1000.millis()).await;
        }
    }
    #[task(local = [foc_pwm])]
    async fn motor_task(cx: motor_task::Context) {
        rprintln!("motor  with sinusoidal driver, 1/4 torque, one turn per second");
        let mut curr_angle = I16F16::ZERO;
        loop {
            curr_angle += I16F16::TAU / 200;
            if curr_angle >= I16F16::TAU {
              curr_angle = I16F16::ZERO;
            }

            // 2. Run the FOC algorithm
            cx.local.foc_pwm
                .update(curr_angle, I16F16::ONE , None)
                .unwrap();
            Mono::delay(1.millis()).await;
        }
    }
}
