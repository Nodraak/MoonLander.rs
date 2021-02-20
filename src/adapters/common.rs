use serde::{Serialize, Deserialize};
use uom::si::f64::*;

use crate::utils::math::Vec2;


pub trait Adapter {
    fn read_sensors(&mut self) -> SensorsValues;
    fn write_actuators(&mut self, control: ActuatorsValues);
    fn export_to_csv_conf(&self);
    fn export_to_csv_cur(&self);
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct SensorsValues {
    pub dt_step: Time,

    // from accelerometers
    pub spacecraft_acc: Vec2<Acceleration>,
    pub spacecraft_ang_acc: AngularAcceleration,
    // from radar altimeter
    pub spacecraft_altitude: Option<Length>,

    // TODO heading
}

/// Note: Sim is 2D, KSP will project on the (velocity vector, local vertical) plane
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct ActuatorsValues {
    pub engine_throttle: Ratio, // range: [0; 1] of max (nominal) thrust
    pub engine_gimbal: Ratio,   // range: [-1; +1] of max gimbal

    // TODO heading
}
