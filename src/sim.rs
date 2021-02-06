use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::conf::{Conf, Scenario};
use crate::spacecraft::SpacecraftDynamic;
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


pub struct Sim {
    dt_step: f64,
    conf: Scenario,                 // spacecraft configuration / static properties
    cur: SpacecraftDynamic,         // latest changing properties
    all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Sim {
    pub fn new(conf: Conf) -> Sim {
        Sim {
            dt_step: conf.dt_step,
            conf: conf.s,
            cur: SpacecraftDynamic::new(&conf.s),
            all: vec![],
        }
    }

    pub fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            dt_step: self.dt_step,
            spacecraft_acc: self.cur.acc,
            spacecraft_ang_acc: self.cur.ang_acc,
            spacecraft_altitude: Some(self.cur.pos.y),
        })
    }

    pub fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let dt = self.dt_step;
        let sc_mass = self.conf.sc_dry_mass + self.cur.fuel_mass;

        // compute t, mass

        let t = self.cur.t + dt;

        let sc_fuel_mass = self.cur.fuel_mass - self.conf.sc_nominal_mass_flow*control.engine_throttle*dt;

        // self.dv_flight += acc*dt

        // compute torque and angular vel/pos

        let sc_moment_of_inertia = 0.5 * sc_mass * (self.conf.sc_height/2.0).powi(2);  // 1/2*m*r**2 = kg.m**2
        let torque = (
            self.conf.sc_height/2.0
            * control.engine_throttle*self.conf.sc_nominal_thrust
            * (control.engine_gimbal*self.conf.ctr_eng_gimbal_pos_max).sin()
        );
        let sc_ang_acc = torque/sc_moment_of_inertia;

        let sc_ang_vel = self.cur.ang_vel + sc_ang_acc*dt;
        let sc_ang_pos = self.cur.ang_pos + sc_ang_vel*dt;

        // compute thrust and acc/vel/pos

        let engine_acc_norm = control.engine_throttle * self.conf.sc_nominal_thrust/sc_mass;
        let engine_acc = Vec2::new_polar(engine_acc_norm, self.cur.ang_pos);
        let gravity_acc = Vec2 {
            x: 0.0,
            y:
                - moon_gravity(self.cur.pos.y)
                + moon_centrifugal(self.cur.vel.x, self.cur.pos.y),
        };
        let sc_acc = engine_acc + gravity_acc;
        // self.g = self.acc_y/G0

        let sc_vel = self.cur.vel + sc_acc*dt;
        let sc_pos = self.cur.pos + sc_vel*dt;

        // save everything

        self.cur.eng_gimbal = control.engine_gimbal;
        self.cur.eng_throttle = control.engine_throttle;

        self.cur.t = t;
        self.cur.fuel_mass = sc_fuel_mass;

        self.cur.ang_pos = sc_ang_pos;
        self.cur.ang_vel = sc_ang_vel;
        self.cur.ang_acc = sc_ang_acc;

        self.cur.pos = sc_pos;
        self.cur.vel = sc_vel;
        self.cur.acc = sc_acc;

        self.all.push(self.cur);

        println!("{:?}", self.cur);

        Ok(())
    }

    pub fn export_to_csv_header(&self) {
        println!("CSV SIM filtering key;tgo;eng_throttle;mass;eng_gimbal;sc_ang_acc;sc_ang_vel;sc_ang_pos;acc_x;acc_y;vel_x;vel_y;pos_x;pos_y;");
    }

    pub fn export_to_csv(&self, tgo: f64) {
        let mass = self.conf.sc_dry_mass + self.cur.fuel_mass;
        println!(
            "CSV SIM;{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};",
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
