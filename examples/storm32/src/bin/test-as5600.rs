//! Ebus Electronics copyright 2024
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::types::I16F16;
use foc_simple::SensorAngle;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};
use storm32::foc_impl::SensorAngleImpl;

bind_interrupts!(struct Irqs {
  I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
  I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;

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
    rtt_init_print!();

    let p = embassy_stm32::init(Default::default());
    rprintln!("Start test as5600 magnetic angle sensors");

    let led0 = Output::new(p.PB12, Level::High, Speed::Medium);
    _ = spawner.spawn(ticker_task(led0));
    rprintln!("All tasks spawned");

    let mut led1 = Output::new(p.PB13, Level::High, Speed::Medium);
    for _i in 0..5 {
        led1.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }

    let i2c1 = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    let i2c2 = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz(100_000), Default::default());
    Timer::after(Duration::from_millis(100)).await;

    // wrap as5600 object to make the SensorAngle interface available
    let mut sensor_angle1 = SensorAngleImpl::from_as5600(i2c1);
    let mut sensor_angle2 = SensorAngleImpl::from_as5600(i2c2);
    // crate the foc_angle object

    Timer::after(Duration::from_millis(100)).await;

    let mut ts = Instant::now().as_micros();
    Timer::after(Duration::from_millis(1)).await;
    loop {
        let now = Instant::now().as_micros();
        let delta_dt = (now - ts).clamp(0, 32767); // be sure the delta fits in I16F16
        ts = now;

        match sensor_angle1.get_angle() {
            Err(_) => led1.set_high(),
            Ok(a) => {
                led1.set_low();
                let angle: f32 = a.to_num();
                rprintln!("1a:{:?}", angle);
            }
        }
        match sensor_angle2.get_angle() {
            Err(_) => led1.set_high(),
            Ok(a) => {
                led1.set_low();
                let angle: f32 = a.to_num();
                rprintln!("2 a:{:?}", angle);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
