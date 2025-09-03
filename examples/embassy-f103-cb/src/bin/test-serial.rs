//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use heapless::String;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_default, rtt_init_print};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);

    let p: embassy_stm32::Peripherals = embassy_stm32::init(Default::default());
    rprintln!("Start basic uart test");
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

    let (mut tx, mut rx) = uart.split();

    let mut count = 0;
    for _x in 0..10 {
        rprintln!("Tick");
        let mut msg: String<64> = String::new();
        core::writeln!(&mut msg, "Hello from embassy  {:02}\r\n", count).unwrap();
        count += 1;
        tx.write(&msg.as_bytes()).await.unwrap();
        tx.flush().await.unwrap();
        Timer::after(Duration::from_millis(100)).await;
    }
    {
        let mut msg: String<64> = String::new();
        core::writeln!(&mut msg, "Hello from embassy  {:02}\r\n", count).unwrap();
        tx.write(&msg.as_bytes()).await.unwrap();
        tx.flush().await.unwrap();
    }

    loop {
        for _x in 0..10 {
            rprintln!("Waiting for user input");
            let mut msg: String<64> = String::new();
            core::writeln!(
                &mut msg,
                " Please send characters to the device. Count nr:{:02}\r\n",
                count
            )
            .unwrap();
            count += 1;
            tx.write(&msg.as_bytes()).await.unwrap();
            let mut buffer = [0_u8; 64];
            match rx.read_until_idle(&mut buffer).await {
                Ok(len) => {
                    tx.write(&&buffer[0..len]).await.unwrap();
                    tx.flush().await.unwrap();
                }
                Err(_) => (),
            }
        }
    }
}
