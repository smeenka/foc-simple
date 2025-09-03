use embassy_futures::select::{select3, Either3};
use embassy_stm32::exti::ExtiInput;
use fixed::types::I16F16;
use foc_simple::foc::foc_embassy::HALL_ANGLES;
use foc_simple::foc::foc_hall_sensor::FocHallSensor;
use foc_simple::Result;
use rtt_target::rprintln;

pub struct HallSensorImpl {
    motor_nr: usize,
    h1: ExtiInput<'static>,
    h2: ExtiInput<'static>,
    h3: ExtiInput<'static>,
    hall_sensor: FocHallSensor,
}

impl HallSensorImpl {
    pub fn new(
        motor_nr: usize,
        h1: ExtiInput<'static>,
        h2: ExtiInput<'static>,
        h3: ExtiInput<'static>,
        nr_poles: usize,
    ) -> Self {
        let hall_sensor = FocHallSensor::new(motor_nr, nr_poles);
        //  hall_table = [1, 3, 2, 6, 4, 5]; // sequence of hall states in table
        HallSensorImpl {
            motor_nr,
            h1,
            h2,
            h3,
            hall_sensor,
        }
    }

    pub fn get_errorcount(&self) -> usize {
        self.hall_sensor.get_errorcount()
    }
    pub fn reset_errorcount_reset(&mut self) {
        self.hall_sensor.reset_errorcount();
    }
    /// async waiting on hall sensor edges
    pub async fn update(&mut self) -> Result<()> {
        let result = match select3(
            self.h1.wait_for_any_edge(),
            self.h2.wait_for_any_edge(),
            self.h3.wait_for_any_edge(),
        )
        .await
        {
            Either3::First(_) => self.handle_update(),
            Either3::Second(_) => self.handle_update(),
            Either3::Third(_) => self.handle_update(),
        };
        match result {
            Ok(angle) => HALL_ANGLES[self.motor_nr].signal(angle),
            Err(_) => rprintln!("Hall sensor error"),
        }
        Ok(())
    }

    fn handle_update(&mut self) -> Result<I16F16> {
        let mut phase = 0;
        if self.h1.is_high() {
            phase |= 1;
        }
        if self.h2.is_high() {
            phase |= 2;
        }
        if self.h3.is_high() {
            phase |= 4;
        }
        self.hall_sensor.update(phase)
    }
}
