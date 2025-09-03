//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Ticker, Timer};
use foc_simple::foc::foc_simple_async::HALL_ANGLES;
use nucleo_f103_cb::foc_impl::hall_sensor_impl::HallSensorImpl;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default};

bind_interrupts!(
    struct Irqs {
          USART2 => usart::InterruptHandler<peripherals::USART2>;

    }
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
pub async fn listen_task(mut led: Output<'static>) {
    loop {
        let angle = HALL_ANGLES[0].wait().await;
        rprintln!("Angle:{}", angle);
        led.toggle();
    }
}

#[embassy_executor::task]
pub async fn hall_sensor_task(mut sensor: HallSensorImpl) {
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        match select(sensor.update(), ticker.next()).await {
            Either::First(_) => (),
            Either::Second(_) => {
                let errors = sensor.get_errorcount();
                rprintln!("Errors:{}  ", errors);
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p = embassy_stm32::init(Default::default());

    let led0 = Output::new(p.PB13, Level::High, Speed::Medium);
    let led1 = Output::new(p.PB2, Level::High, Speed::Medium);

    let _uart = Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        Config::default(),
    )
    .unwrap();

    Timer::after(Duration::from_millis(100)).await;
    rprintln!("Start test for reading hall sensors low level");

    // PB3 has problems, is not routed to the CPU for whatever reason
    // PB5 is connected to PB3 with a jumper
    let _h2_orig = Input::new(p.PB3, Pull::None);

    let h1 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);
    let h2 = ExtiInput::new(p.PB5, p.EXTI5, Pull::Down);
    let h3 = ExtiInput::new(p.PB10, p.EXTI10, Pull::Down);
    let sensor = HallSensorImpl::new(0, h3, h2, h1, 6, false);

    _ = spawner.spawn(ticker_task(led0));
    _ = spawner.spawn(listen_task(led1));
    _ = spawner.spawn(hall_sensor_task(sensor));

    rprintln!("All tasks spawned");
    let mut ticker = Ticker::every(Duration::from_secs(30));
    loop {
        ticker.next().await;
        rprintln!("Hello, world!");
    }
}
