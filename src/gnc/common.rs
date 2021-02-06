use std::f64::consts::PI;

use crate::conf::Scenario;
use crate::spacecraft::SpacecraftDynamic;


pub struct Spacecraft {
    pub conf: Scenario,                 // spacecraft configuration / static properties
    pub cur: SpacecraftDynamic,         // latest changing properties
    pub all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new(conf: Scenario) -> Spacecraft {
        Spacecraft {
            conf: conf,
            cur: SpacecraftDynamic::new(&conf),
            all: vec![],
        }
    }

    /// Some sanity checks to ensure we dont continue with stupid values
    pub fn check_state(&self) {
        assert!(0.0 < self.cur.t);
        assert!(self.cur.t < 15.0*60.0);

        assert!(-1.0 <= self.cur.eng_gimbal);
        assert!(self.cur.eng_gimbal <= 1.0);

        assert!(0.0 <= self.cur.eng_throttle);
        assert!(self.cur.eng_throttle <= 1.0);

        assert!(0.0 <= self.cur.fuel_mass);
        assert!(self.cur.fuel_mass <= self.conf.initial_sc_fuel_mass);

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

        assert!(-1.0*PI/180.0 <= self.cur.ang_acc);
        assert!(self.cur.ang_acc <= 1.0*PI/180.0);
    }

    pub fn export_to_csv_header(&self) {
        println!("CSV SC filtering key;tgo;eng_throttle;mass;eng_gimbal;sc_ang_acc;sc_ang_vel;sc_ang_pos;acc_x;acc_y;vel_x;vel_y;pos_x;pos_y;");
    }

    pub fn export_to_csv(&self, tgo: f64) {
        let mass = self.conf.sc_dry_mass + self.cur.fuel_mass;
        println!(
            "CSV SC;{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};",
            tgo,
            self.cur.eng_throttle,
            mass,
            self.cur.eng_gimbal*self.conf.ctr_eng_gimbal_pos_max,
            self.cur.ang_acc,
            self.cur.ang_vel,
            self.cur.ang_pos,
            self.cur.acc.x, self.cur.acc.y,
            self.cur.vel.x, self.cur.vel.y,
            self.cur.pos.x, self.cur.pos.y,
        );
    }
}
