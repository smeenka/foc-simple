//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{adc, bind_interrupts};
use embassy_time::{Duration, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::foc::foc_pwm::FocPwm;
use foc_simple::foc::EModulation;
use embassy_f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
});

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
pub async fn motor_task(mut foc_pwm: FocPwm<PwmDriverImpl<'static>>) {
    rprintln!("motor  with sinusoidal driver, 1/4 torque, one turn per  second");
    let mut curr_angle = I16F16::ZERO;
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        curr_angle += 3 * 6 * I16F16::ONE / 1000;

        // 2. Run the FOC algorithm
        foc_pwm.update(curr_angle, I16F16::ONE / 4, None).unwrap();
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Test pwm driver");

    let p = embassy_stm32::init(Default::default());

    let a0 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let b0 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let c0 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

    let ea0 = Output::new(p.PC10, Level::High, Speed::Medium);
    let eb0 = Output::new(p.PC11, Level::High, Speed::Medium);
    let ec0 = Output::new(p.PC12, Level::High, Speed::Medium);

    let led0 = Output::new(p.PB13, Level::High, Speed::Medium);

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

    let max = pwm_driver.get_max_duty();

    let foc_pwm = FocPwm::new(pwm_driver, EModulation::Sinusoidal, max, 0.3);
    _ = spawner.spawn(motor_task(foc_pwm));
    _ = spawner.spawn(ticker_task(led0));
    rprintln!("All tasks spawned");

    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
