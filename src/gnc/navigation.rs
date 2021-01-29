use crate::adapters::common::SensorsValues;
use crate::gnc::common::Spacecraft;


pub fn nav(spacecraft: &mut Spacecraft, sensors_vals: SensorsValues) {
    let dt = sensors_vals.dt_step;

    spacecraft.cur.t += dt;

    spacecraft.cur.fuel_mass -= spacecraft.spec.nominal_mass_flow*spacecraft.cur.eng_throttle*dt;

    spacecraft.cur.acc = sensors_vals.spacecraft_acc;
    spacecraft.cur.vel += spacecraft.cur.acc*dt;
    spacecraft.cur.pos += spacecraft.cur.vel*dt;

    spacecraft.cur.ang_vel += sensors_vals.spacecraft_ang_acc*dt;
    spacecraft.cur.ang_pos += spacecraft.cur.ang_vel*dt;

    // TODO cross check altitude - kalman filter?

    // save everything
    spacecraft.all.push(spacecraft.cur);
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nav_1() {
        let mut sc = Spacecraft::new();

        nav(&mut sc, SensorsValues {
            spacecraft_acc: Vec2 {x: 1.0, y: 2.0},
            spacecraft_altitude: None,
        });
        assert_eq!(sc.cur.acc.x, 1.0);
        assert_eq!(sc.cur.vel.x, 1_673.0+1.0);
        assert_eq!(sc.cur.pos.x, 1_673.0+1.0);
        assert_eq!(sc.cur.acc.y, 2.0);
        assert_eq!(sc.cur.vel.y, 2.0);
        assert_eq!(sc.cur.pos.y, 15_000.0+2.0);

        nav(&mut sc, SensorsValues {
            spacecraft_acc: Vec2 {x: 3.0, y: 4.0},
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
