use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use foc_simple::foc::PwmDriver;
use foc_simple::Result;
use rtt_target::rprintln;

pub struct PwmDriverImpl<'a> {
    pwm_a: SimplePwmChannel<'a, TIM1>,
    pwm_b: SimplePwmChannel<'a, TIM1>,
    pwm_c: SimplePwmChannel<'a, TIM1>,
    enable_a: Output<'static>,
    enable_b: Output<'static>,
    enable_c: Output<'static>,
}

impl PwmDriverImpl<'static> {
    pub fn new(
        pwm: SimplePwm<'static, TIM1>,
        enable_a: Output<'static>,
        enable_b: Output<'static>,
        enable_c: Output<'static>,
    ) -> Self {
        let mut channels1 = pwm.split();
        channels1.ch1.enable();
        channels1.ch2.enable();
        channels1.ch3.enable();

        rprintln!("Max duty: {:?}", channels1.ch1.max_duty_cycle());

        PwmDriverImpl {
            pwm_a: channels1.ch1,
            pwm_b: channels1.ch2,
            pwm_c: channels1.ch3,
            enable_a,
            enable_b,
            enable_c,
        }
    }
    pub fn get_max_duty(&self) -> u16 {
      return self.pwm_a.max_duty_cycle()
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

impl PwmDriver for PwmDriverImpl<'static> {
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
                0 => self.pwm_a.set_duty_cycle(phase),
                1 => self.pwm_b.set_duty_cycle(phase),
                2 => self.pwm_c.set_duty_cycle(phase),
                _ => (),
            }
        }
        Ok(())
    }
}
