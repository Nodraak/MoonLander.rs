use crate::adapters::common::SensorsValues;
use crate::gnc::common::Spacecraft;

pub fn nav(spacecraft: &mut Spacecraft, sensors_vals: SensorsValues) {
    spacecraft.cur.acc = sensors_vals.spacecraft_acc;

    spacecraft.cur.vel += spacecraft.cur.acc;
    spacecraft.cur.pos += spacecraft.cur.vel;

    // TODO cross check altitude - kalman filter?
}
