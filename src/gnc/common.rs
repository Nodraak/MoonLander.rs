use std::f64::consts::PI;

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

        assert!(0.0 <= self.cur.t);
        assert!(self.cur.t < 15.0*60.0);

        assert!(-1.0 <= self.cur.eng_gimbal);
        assert!(self.cur.eng_gimbal <= 1.0);

        assert!(0.0 <= self.cur.eng_throttle);
        assert!(self.cur.eng_throttle <= 1.0);

        assert!(0.0 <= self.cur.fuel_mass);
        assert!(self.cur.fuel_mass <= self.conf.s.initial_sc_fuel_mass);

        assert!(0.0 <= self.cur.pos.y);
        assert!(self.cur.pos.y <= 1_000_000.0);

        assert!(self.cur.vel.x.abs() < 10_000.0);
        assert!(self.cur.vel.y.abs() < 1_000.0);

        assert!(self.cur.acc.x.abs() < 100.0);
        assert!(self.cur.acc.y.abs() < 100.0);

        assert!(-180.1*PI/180.0 <= self.cur.ang_pos);
        assert!(self.cur.ang_pos <= 180.1*PI/180.0);

        assert!(-5.0*PI/180.0 <= self.cur.ang_vel);
        assert!(self.cur.ang_vel <= 5.0*PI/180.0);

        assert!(-5.0*PI/180.0 <= self.cur.ang_acc);
        assert!(self.cur.ang_acc <= 5.0*PI/180.0);
    }

    pub fn export_to_csv_conf(&self) {
        println!("[LOGD:Spacecraft::export_to_csv_conf] CSV={}", serde_json::to_string(&self.conf.s).unwrap());
    }

    pub fn export_to_csv_cur(&self) {
        println!("[LOGD:Spacecraft::export_to_csv_cur] CSV={}", serde_json::to_string(&self.cur).unwrap());
    }
}
