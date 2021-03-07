use std::f64::consts::PI;
use uom::si::f64::*;
use uom::si::angle::degree;
use uom::si::velocity::meter_per_second;

use crate::{mul, norm, sqrt, squared};
use crate::adapters::common::SensorsValues;
use crate::gnc::common::Spacecraft;
use crate::utils::math::Vec2;


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
    let vel: Velocity = Velocity::new::<meter_per_second>(norm!(spacecraft.cur.vel));
    let dp_q: Pressure = 0.5 * conf.body.atmosphere_density(spacecraft.cur.pos.y) * squared!(vel);
    let dp_n: Force = dp_q * (PI*squared!(conf.sc_width/2.0)) * conf.sc_cd;
    spacecraft.cur.acc_atm = -dp_n/sc_mass;

    spacecraft.cur.acc_gravity = -conf.body.gravity(spacecraft.cur.pos.y);
    spacecraft.cur.acc_centrifugal = conf.body.centrifugal(spacecraft.cur.vel.x, spacecraft.cur.pos.y);

    spacecraft.cur.acc = sensors_vals.spacecraft_acc;
    spacecraft.cur.vel += mul!(spacecraft.cur.acc, dt);
    spacecraft.cur.pos += mul!(spacecraft.cur.vel, dt);

    spacecraft.cur.dv += spacecraft.cur.acc_thrust*dt;

    spacecraft.cur.ang_acc = sensors_vals.spacecraft_ang_acc;
    let dav: AngularVelocity = (spacecraft.cur.ang_acc*dt).into();
    spacecraft.cur.ang_vel += dav;
    let dap: Angle = (spacecraft.cur.ang_vel*dt).into();
    spacecraft.cur.ang_pos += dap;

    while spacecraft.cur.ang_pos > Angle::new::<degree>(180.0) {
        spacecraft.cur.ang_pos -= Angle::new::<degree>(360.0);
    }
    while spacecraft.cur.ang_pos < -Angle::new::<degree>(180.0) {
        spacecraft.cur.ang_pos += Angle::new::<degree>(360.0);
    }

    // TODO cross check altitude - kalman filter?

    // save everything
    spacecraft.all.push(spacecraft.cur);
}


#[cfg(test)]
mod tests {
    use super::*;

    use serde_yaml;
    use uom::si::acceleration::meter_per_second_squared;
    use uom::si::angular_acceleration::degree_per_second_squared;
    use uom::si::length::meter;
    use uom::si::time::second;

    use crate::utils::math::Vec2;
    use crate::conf::{SubCommand, Scenario, Conf};

    #[test]
    fn test_nav_1() {
        let f = std::fs::File::open("conf/Apollo-descent.yaml").unwrap();  // TODO conf file for tests
        let scenario: Scenario = serde_yaml::from_reader(f).unwrap();
        let conf = Conf::new(SubCommand::Sim, Time::new::<second>(1.0), Time::new::<second>(1.0), scenario);

        let mut sc = Spacecraft::new(conf);
        sc.cur.pos.x = Length::new::<meter>(0.0);  // TODO conf file for tests

        nav(&mut sc, &SensorsValues {
            dt_step: Time::new::<second>(1.0),
            spacecraft_acc: Vec2 {
                x: Acceleration::new::<meter_per_second_squared>(1.0),
                y: Acceleration::new::<meter_per_second_squared>(2.0),
            },
            spacecraft_ang_acc: AngularAcceleration::new::<degree_per_second_squared>(0.0),
            spacecraft_altitude: None,
        });
        assert_eq!(sc.cur.acc.x, Acceleration::new::<meter_per_second_squared>(1.0));
        assert_eq!(sc.cur.vel.x, Velocity::new::<meter_per_second>(1_673.0+1.0));
        assert_eq!(sc.cur.pos.x, Length::new::<meter>(1_673.0+1.0));
        assert_eq!(sc.cur.acc.y, Acceleration::new::<meter_per_second_squared>(2.0));
        assert_eq!(sc.cur.vel.y, Velocity::new::<meter_per_second>(2.0));
        assert_eq!(sc.cur.pos.y, Length::new::<meter>(15_000.0+2.0));

        nav(&mut sc, &SensorsValues {
            dt_step: Time::new::<second>(1.0),
            spacecraft_acc: Vec2 {
                x: Acceleration::new::<meter_per_second_squared>(3.0),
                y: Acceleration::new::<meter_per_second_squared>(4.0),
            },
            spacecraft_ang_acc: AngularAcceleration::new::<degree_per_second_squared>(0.0),
            spacecraft_altitude: None,
        });
        assert_eq!(sc.cur.acc.x, Acceleration::new::<meter_per_second_squared>(3.0));
        assert_eq!(sc.cur.vel.x, Velocity::new::<meter_per_second>(1_673.0 + 1.0 + 3.0));
        assert_eq!(sc.cur.pos.x, Length::new::<meter>(1_673.0+1.0 + 1_673.0+1.0+3.0));
        assert_eq!(sc.cur.acc.y, Acceleration::new::<meter_per_second_squared>(4.0));
        assert_eq!(sc.cur.vel.y, Velocity::new::<meter_per_second>(2.0 + 4.0));
        assert_eq!(sc.cur.pos.y, Length::new::<meter>(15_000.0 + 2.0 + 2.0+4.0));
    }
}
