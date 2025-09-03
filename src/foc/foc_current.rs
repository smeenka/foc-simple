use crate::SensorCurrent;

/// calculate the foc currents from the sensor values
pub struct FocCurrent<C: SensorCurrent> {
  sensor_current: C,
}

impl<C> FocCurrent<C>
where
  C: SensorCurrent,
{
  pub fn new(sensor_current: C) -> Self {
    FocCurrent { sensor_current }
  }
}
