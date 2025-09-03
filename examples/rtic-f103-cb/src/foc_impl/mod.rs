pub mod pwm_driver;

use core::future::Future;
use fixed::types::I16F16;
use foc_simple::SensorAngle;
use foc_simple::{EFocSimpleError, FocSerial, Result, SensorCurrent};
use stm32f1xx_hal::serial::Rx;
use stm32f1xx_hal::{pac::USART2, serial::Tx};

pub struct FocSerialImpl {
    tx: Tx<USART2>,
    rx: Rx<USART2>,
}

impl FocSerialImpl {
    pub fn new(tx: Tx<USART2>, rx: Rx<USART2>) -> Self {
        FocSerialImpl { tx, rx }
    }
    async fn wait_receive(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let mut idx = 0;
        while self.rx.is_rx_not_empty() {
            match self.rx.read() {
                Ok(c) => {
                    buffer[idx] = c;
                    idx += 1;
                }
                Err(_err) => (),
            }
        }
        Ok(idx)
    }
}

impl FocSerial for FocSerialImpl {
    fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize>> {
        // return the function, do not await here
        self.wait_receive(buffer)
    }

    fn send(&mut self, buffer: &[u8]) -> Result<()> {
        match self.tx.bwrite_all(buffer) {
            Ok(()) => Ok(()),
            Err(_) => Err(EFocSimpleError::SerialError),
        }
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
pub struct SensorAngleDummy {}

impl SensorAngleDummy {
    pub fn new() -> Self {
        SensorAngleDummy {}
    }
}

impl SensorAngle for SensorAngleDummy {
    fn get_angle(&mut self) -> Result<I16F16> {
        Ok(I16F16::ZERO)
    }
}
