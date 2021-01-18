use crate::utils::math::Vec2;

pub trait Adapter {
    fn read_sensors(&self) -> Result<SensorsValues, &'static str>;
    fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str>;
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
pub struct SensorsValues {
    pub spacecraft_acc: Vec2,               // unit: m/s
    pub spacecraft_altitude: Option<f64>,   // unit: m - from radar altimeter
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
pub struct ActuatorsValues {
    pub engine_gimbal: f64,     // unit: rad
    pub engine_throttle: f64,   // unit: 0-1
}
