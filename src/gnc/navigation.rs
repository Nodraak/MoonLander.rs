use crate::adapters::common::SensorsValues;
use crate::gnc::common::Spacecraft;
use crate::utils::math::Vec2;


pub fn nav(spacecraft: &mut Spacecraft, sensors_vals: SensorsValues) {
    spacecraft.cur.acc = sensors_vals.spacecraft_acc;

    spacecraft.cur.vel += spacecraft.cur.acc;
    spacecraft.cur.pos += spacecraft.cur.vel;

    // TODO cross check altitude - kalman filter?
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
