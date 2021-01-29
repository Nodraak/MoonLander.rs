use crate::spacecraft::{SpacecraftStatic, SpacecraftDynamic};


pub struct Spacecraft {
    pub spec: SpacecraftStatic,         // fixed properties
    pub cur: SpacecraftDynamic,         // latest changing properties
    pub all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new() -> Spacecraft {
        Spacecraft {
            spec: SpacecraftStatic::new(),
            cur: SpacecraftDynamic::new(),
            all: vec![],
        }
    }

    pub fn export_to_csv_header(&self) {
        println!("CSV SC filtering key;tgo;eng_throttle;mass;eng_gimbal;sc_ang_acc;sc_ang_vel;sc_ang_pos;acc_x;acc_y;vel_x;vel_y;pos_x;pos_y;");
    }

    pub fn export_to_csv(&self, tgo: f64) {
        let mass = self.spec.dry_mass + self.cur.fuel_mass;
        println!(
            "CSV SC;{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};",
            tgo,
            self.cur.eng_throttle,
            mass,
            self.cur.eng_gimbal,
            self.cur.ang_acc,
            self.cur.ang_vel,
            self.cur.ang_pos,
            self.cur.acc.x, self.cur.acc.y,
            self.cur.vel.x, self.cur.vel.y,
            self.cur.pos.x, self.cur.pos.y,
        );
    }
}
