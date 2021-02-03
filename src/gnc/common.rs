use crate::conf::Conf;
use crate::spacecraft::SpacecraftDynamic;


pub struct Spacecraft {
    pub conf: Conf,                     // spacecraft configuration / static properties
    pub cur: SpacecraftDynamic,         // latest changing properties
    pub all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new(conf: Conf) -> Spacecraft {
        Spacecraft {
            conf: conf,
            cur: SpacecraftDynamic::new(&conf),
            all: vec![],
        }
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
