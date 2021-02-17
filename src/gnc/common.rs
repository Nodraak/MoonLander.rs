use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::angular_acceleration::degree_per_second_squared;
use uom::si::angular_velocity::degree_per_second;
use uom::si::length::meter;
use uom::si::mass::kilogram;
use uom::si::ratio::ratio;
use uom::si::time::{minute, second};
use uom::si::velocity::meter_per_second;

use crate::conf::{Conf, SubCommand};
use crate::utils::spacecraft::SpacecraftDynamic;


pub struct Spacecraft {
    pub conf: Conf,                     // spacecraft configuration / static properties
    pub cur: SpacecraftDynamic,         // latest changing properties
    pub all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new(conf: Conf) -> Spacecraft {
        Spacecraft {
            conf: conf,
            cur: SpacecraftDynamic::new(&conf.s),
            all: vec![],
        }
    }

    /// Some sanity checks to ensure we dont continue with stupid values
    pub fn check_state(&self) {
        if self.conf.subcommand == SubCommand::Ksp {
            return;
        }

        assert!(Time::new::<second>(0.0) <= self.cur.t);
        assert!(self.cur.t < Time::new::<minute>(15.0));

        assert!(Ratio::new::<ratio>(-1.0) <= self.cur.eng_gimbal);
        assert!(self.cur.eng_gimbal <= Ratio::new::<ratio>(1.0));

        assert!(Ratio::new::<ratio>(0.0) <= self.cur.eng_throttle);
        assert!(self.cur.eng_throttle <= Ratio::new::<ratio>(1.0));

        assert!(Mass::new::<kilogram>(0.0) <= self.cur.fuel_mass);
        assert!(self.cur.fuel_mass <= self.conf.s.initial_sc_fuel_mass);

        assert!(Length::new::<meter>(0.0) <= self.cur.pos.y);
        assert!(self.cur.pos.y <= Length::new::<meter>(1_000_000.0));

        assert!(self.cur.vel.x.abs() < Velocity::new::<meter_per_second>(10_000.0));
        assert!(self.cur.vel.y.abs() < Velocity::new::<meter_per_second>(1_000.0));

        assert!(self.cur.acc.x.abs() < Acceleration::new::<meter_per_second_squared>(100.0));
        assert!(self.cur.acc.y.abs() < Acceleration::new::<meter_per_second_squared>(100.0));

        assert!(Angle::new::<degree>(-180.1) <= self.cur.ang_pos);
        assert!(self.cur.ang_pos <= Angle::new::<degree>(180.1));

        assert!(AngularVelocity::new::<degree_per_second>(-5.0) <= self.cur.ang_vel);
        assert!(self.cur.ang_vel <= AngularVelocity::new::<degree_per_second>(5.0));

        assert!(AngularAcceleration::new::<degree_per_second_squared>(-5.0) <= self.cur.ang_acc);
        assert!(self.cur.ang_acc <= AngularAcceleration::new::<degree_per_second_squared>(5.0));
    }

    pub fn export_to_csv_conf(&self) {
        println!("[LOGD:Spacecraft::export_to_csv_conf] CSV={}", serde_json::to_string(&self.conf.s).unwrap());
    }

    pub fn export_to_csv_cur(&self) {
        println!("[LOGD:Spacecraft::export_to_csv_cur] CSV={}", serde_json::to_string(&self.cur).unwrap());
    }
}
