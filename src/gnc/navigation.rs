use std::f64::consts::PI;

use crate::adapters::common::SensorsValues;
use crate::gnc::common::Spacecraft;


pub fn nav(spacecraft: &mut Spacecraft, sensors_vals: &SensorsValues) {
    let conf = spacecraft.conf.s;
    let dt = sensors_vals.dt_step;
    let sc_mass = conf.sc_dry_mass + spacecraft.cur.fuel_mass;

    spacecraft.cur.t += dt;
    spacecraft.cur.dt = dt;

    spacecraft.cur.fuel_mass -= conf.sc_nominal_mass_flow*spacecraft.cur.eng_throttle*dt;

    spacecraft.cur.acc_thrust = (spacecraft.cur.eng_throttle*conf.sc_nominal_thrust)/sc_mass;
    // dynamic pressure q: Pa = Kg/(m*s**2)
    // dynamic pressure n: N = Kg/(m*s**2) * m**2 = Kg*m/(s**2)
    let dp_q = 0.5 * conf.body.atmosphere_density(spacecraft.cur.pos.y) * spacecraft.cur.vel.norm().powi(2);
    let dp_n = dp_q * (PI*(conf.sc_width/2.0).powi(2)) * conf.sc_cd;
    spacecraft.cur.acc_atm = -dp_n/sc_mass;

    spacecraft.cur.acc_gravity = -conf.body.gravity(spacecraft.cur.pos.y);
    spacecraft.cur.acc_centrifugal = conf.body.centrifugal(spacecraft.cur.vel.x, spacecraft.cur.pos.y);

    spacecraft.cur.acc = sensors_vals.spacecraft_acc;
    spacecraft.cur.vel += spacecraft.cur.acc*dt;
    spacecraft.cur.pos += spacecraft.cur.vel*dt;

    spacecraft.cur.dv += spacecraft.cur.acc_thrust*dt;

    spacecraft.cur.ang_vel += sensors_vals.spacecraft_ang_acc*dt;
    spacecraft.cur.ang_pos += spacecraft.cur.ang_vel*dt;

    // TODO cross check altitude - kalman filter?

    // save everything
    spacecraft.all.push(spacecraft.cur);
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::Vec2;

    #[test]
    fn test_nav_1() {
        let mut sc = Spacecraft::new();

        nav(&mut sc, &SensorsValues {
            dt_step: 1.0,
            spacecraft_acc: Vec2 {x: 1.0, y: 2.0},
            spacecraft_ang_acc: 0.0,
            spacecraft_altitude: None,
        });
        assert_eq!(sc.cur.acc.x, 1.0);
        assert_eq!(sc.cur.vel.x, 1_673.0+1.0);
        assert_eq!(sc.cur.pos.x, 1_673.0+1.0);
        assert_eq!(sc.cur.acc.y, 2.0);
        assert_eq!(sc.cur.vel.y, 2.0);
        assert_eq!(sc.cur.pos.y, 15_000.0+2.0);

        nav(&mut sc, &SensorsValues {
            dt_step: 1.0,
            spacecraft_acc: Vec2 {x: 3.0, y: 4.0},
            spacecraft_ang_acc: 0.0,
            spacecraft_altitude: None,
        });
        assert_eq!(sc.cur.acc.x, 3.0);
        assert_eq!(sc.cur.vel.x, 1_673.0 + 1.0 + 3.0);
        assert_eq!(sc.cur.pos.x, 1_673.0+1.0 + 1_673.0+1.0+3.0);
        assert_eq!(sc.cur.acc.y, 4.0);
        assert_eq!(sc.cur.vel.y, 2.0 + 4.0);
        assert_eq!(sc.cur.pos.y, 15_000.0 + 2.0 + 2.0+4.0);
    }
}
