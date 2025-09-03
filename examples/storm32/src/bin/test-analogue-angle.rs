//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{bind_interrupts };
use embassy_time::{Duration, Ticker, Timer};
use rtt_target::{rprintln, rtt_init_default};
use embassy_stm32::adc::InterruptHandler;
use embassy_stm32::peripherals::ADC1;
use panic_probe as _;

bind_interrupts!(struct Irqs {
  ADC1_2 => InterruptHandler<ADC1>;

});

#[embassy_executor::task]
pub async fn ticker_task(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(300));
    loop {
        led.toggle();
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);  
    
    let mut p = embassy_stm32::init(Default::default());
    rprintln!("Start test as5600 magnetic angle sensors");

    let led0 = Output::new(p.PB12, Level::High, Speed::Medium);
    _ = spawner.spawn(ticker_task(led0));
    rprintln!("All tasks spawned");

    let mut led1 = Output::new(p.PB13, Level::High, Speed::Medium);
    for _i in 0..5 {
        led1.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
    let mut adc = Adc::new(p.ADC1);

    Timer::after(Duration::from_millis(100)).await;

    // wrap as5600 object to make the SensorAngle interface available
    //let sensor_angle1 = SensorAnalogue::new();

    let mut counter = 0;
    Timer::after(Duration::from_millis(100)).await;

    Timer::after(Duration::from_millis(1)).await;
    loop {
        counter += 1;
        if counter % 100 == 0 {
            //let raw_voltage = sensor_angle.get_adc_value();
            let raw_voltage = adc.read(&mut p.PC0).await;
            let angle = 0;
            rprintln!("val:r:{:?} \ta:{:?}", raw_voltage, angle);
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}
