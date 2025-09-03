//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Ticker, Timer};
use foc_simple::FocCommand;
use nucleo_f103_cb::foc_impl::FocSerialImpl;
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
pub async fn command_task(mut command: FocCommand<FocSerialImpl>) {
    command.task().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p = embassy_stm32::init(Default::default());
    rprintln!("Start test foc velociy test");

    // Storm
    //let led0 = Output::new(p.PB12, Level::High, Speed::Medium);

    // nucleo
    let led0 = Output::new(p.PB13, Level::High, Speed::Medium);

    //let mut xor = Output::new(p.PC4, Level::High, Speed::Medium);
    //xor.set_low();

    // Nucleo
    let uart = Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        Config::default(),
    )
    .unwrap();

    Timer::after(Duration::from_millis(2000)).await;
    rprintln!("test shell without drivers started.");

    let parser = FocSerialImpl::new(uart);
    let command = FocCommand::new(parser);

    _ = spawner.spawn(command_task(command));
    _ = spawner.spawn(ticker_task(led0));
    rprintln!("All tasks spawned");
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
