use core::future::Future;

use as5600::As5600;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::peripherals::{TIM2, TIM3, TIM4};
use embassy_stm32::usart::{Uart, UartRx, UartTx};
use embassy_stm32::{i2c::I2c, timer::simple_pwm::SimplePwmChannel};
use fixed::types::I16F16;
use foc_simple::foc::PwmDriver;
use foc_simple::{EFocSimpleError, FocSerial, Result, SensorAngle, SensorCurrent};

pub struct FocSerialImpl {
    tx: UartTx<'static, Async>,
    rx: UartRx<'static, Async>,
}

impl FocSerialImpl {
    pub fn new(uart: Uart<'static, Async>) -> Self {
        let (tx, rx) = uart.split();
        FocSerialImpl { tx, rx }
    }
    async fn wait_receive(&mut self, buffer: &mut [u8]) -> Result<usize> {
        match self.rx.read_until_idle(buffer).await {
            Ok(nr) => Ok(nr),
            Err(_) => Err(EFocSimpleError::SerialError),
        }
    }
}

impl FocSerial for FocSerialImpl {
    fn receive(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize>> {
        // return the future function (do not await here!)
        self.wait_receive(buf)
    }
    fn send(&mut self, buffer: &[u8]) -> Result<()> {
        match self.tx.blocking_write(buffer) {
            Ok(()) => Ok(()),
            Err(_) => Err(EFocSimpleError::SerialError),
        }
    }
}

pub struct SensorAngleImpl {
    sensor: As5600<I2c<'static, Blocking>>,
}

impl SensorAngleImpl {
    pub fn new(sensor: As5600<I2c<'static, Blocking>>) -> Self {
        SensorAngleImpl { sensor }
    }
    pub fn from_as5600(i2c: I2c<'static, Blocking>) -> Self {
        let as5600 = As5600::new(i2c);
        // wrap as5600 object to make the SensorAngle interface available
        SensorAngleImpl::new(as5600)
    }
}

impl SensorAngle for SensorAngleImpl {
    fn get_angle(&mut self) -> Result<I16F16> {
        match self.sensor.angle() {
            Err(_) => Err(EFocSimpleError::AngleSensorError),
            Ok(a) => {
                let angle = ((a as i32) * I16F16::TAU) / 4096;
                Ok(angle)
            }
        }
    }
}
struct SensorAnalogue {}

impl SensorAnalogue {
    fn new() -> Self {
        SensorAnalogue {}
    }
}

impl SensorAngle for SensorAnalogue {
    fn get_angle(&mut self) -> Result<I16F16> {
        Ok(I16F16::ZERO)
    }
}
// dummy implementation
pub struct SensorCurrentImpl {}

impl SensorCurrentImpl {
    pub fn new() -> Self {
        SensorCurrentImpl {}
    }
}
impl SensorCurrent for SensorCurrentImpl {
    fn get_current(&mut self) -> Result<[I16F16; 3]> {
        Ok([I16F16::ZERO; 3])
    }
}

pub struct LeftDriverImpl<'a> {
    a: SimplePwmChannel<'a, TIM3>,
    b: SimplePwmChannel<'a, TIM3>,
    c: SimplePwmChannel<'a, TIM3>,
}

impl LeftDriverImpl<'static> {
    pub fn new(
        mut a: SimplePwmChannel<'static, TIM3>,
        mut b: SimplePwmChannel<'static, TIM3>,
        mut c: SimplePwmChannel<'static, TIM3>,
    ) -> Self {
        a.enable();
        b.enable();
        c.enable();
        LeftDriverImpl { a, b, c }
    }
}

impl PwmDriver for LeftDriverImpl<'static> {
    fn set_pwm(&mut self, phases: [u16; 3]) -> Result<()> {
        for i in 0..3 {
            let phase = phases[i];
            //if phase == 0 {
            //    phase = 1;
            // }
            match i {
                0 => self.a.set_duty_cycle(phase),
                1 => self.b.set_duty_cycle(phase),
                2 => self.c.set_duty_cycle(phase),
                _ => (),
            }
        }
        Ok(())
    }
}

pub struct RightDriverImpl<'a> {
    a: SimplePwmChannel<'a, TIM2>,
    b: SimplePwmChannel<'a, TIM4>,
    c: SimplePwmChannel<'a, TIM4>,
}

impl RightDriverImpl<'static> {
    pub fn new(
        mut a: SimplePwmChannel<'static, TIM2>,
        mut b: SimplePwmChannel<'static, TIM4>,
        mut c: SimplePwmChannel<'static, TIM4>,
    ) -> Self {
        a.enable();
        b.enable();
        c.enable();
        RightDriverImpl { a, b, c }
    }
}

impl PwmDriver for RightDriverImpl<'static> {
    fn set_pwm(&mut self, phases: [u16; 3]) -> Result<()> {
        for i in 0..3 {
            let phase = phases[i];
            //if phase == 0 {
            //    phase = 1;
            //}
            //if phase > self.max_duty {
            //    phase = self.max_duty
            // }
            match i {
                0 => self.a.set_duty_cycle(phase),
                1 => self.b.set_duty_cycle(phase),
                2 => self.c.set_duty_cycle(phase),
                _ => (),
            }
        }
        Ok(())
    }
}
