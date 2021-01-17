pub trait Adapter {
    fn read_sensors(&self) -> Result<SensorsValues, &'static str>;
    fn write_actuators(&self, values: ActuatorsValues) -> Result<(), &'static str>;
    fn tick(&self);
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
pub struct SensorsValues {
    pub spacecraft_acc: (f64, f64),        // TODO vector
    pub spacecraft_altitude: Option<f64>,  // from radar altimeter
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
pub struct ActuatorsValues {
    pub engine_gimbal: f64,
    pub engine_throttle: f64,
}
