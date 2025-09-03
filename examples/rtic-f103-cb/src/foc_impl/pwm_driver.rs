use foc_simple::foc::PwmDriver;
use foc_simple::Result;
use rtt_target::rprintln;
use stm32f1xx_hal::gpio::{ErasedPin, Output};
use stm32f1xx_hal::pac::TIM1;
use stm32f1xx_hal::timer::PwmChannel;

pub struct PwmDriverImpl {
  pwm_a: PwmChannel<TIM1, 0>,
  pwm_b: PwmChannel<TIM1, 1>,
  pwm_c: PwmChannel<TIM1, 2>,
  enable_a: ErasedPin<Output>,
  enable_b: ErasedPin<Output>,
  enable_c: ErasedPin<Output>,
}

impl PwmDriverImpl {
  pub fn new(pwm_a: PwmChannel<TIM1, 0>, pwm_b: PwmChannel<TIM1, 1>, pwm_c: PwmChannel<TIM1, 2>, enable_a: ErasedPin<Output>, enable_b: ErasedPin<Output>, enable_c: ErasedPin<Output>) -> Self {
    rprintln!("Max duty: {:?}", pwm_a.get_max_duty());

    PwmDriverImpl {
      pwm_a,
      pwm_b,
      pwm_c,
      enable_a,
      enable_b,
      enable_c,
    }
  }

  pub fn enable(&mut self) {
    self.enable_a.set_high();
    self.enable_b.set_high();
    self.enable_c.set_high();
  }
  pub fn disable(&mut self) {
    self.enable_a.set_low();
    self.enable_b.set_low();
    self.enable_c.set_low();
  }
}

impl PwmDriver for PwmDriverImpl {
  fn set_pwm(&mut self, phases: [u16; 3]) -> Result<()> {
    //rprintln!("a:{} \tb:{} \tc:{}", phases[0], phases[1], phases[2]);
    self.pwm_a.set_duty(phases[0]);
    self.pwm_b.set_duty(phases[1]);
    self.pwm_c.set_duty(phases[2]);
    Ok(())
  }
}
