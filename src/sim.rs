use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::spacecraft::{SpacecraftStatic, SpacecraftDynamic};
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


pub struct Sim {
    spec: SpacecraftStatic,         // fixed properties
    cur: SpacecraftDynamic,         // latest changing properties
    all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Sim {
    pub fn new() -> Sim {
        Sim {
            spec: SpacecraftStatic::new(),
            cur: SpacecraftDynamic::new(),
            all: vec![],
        }
    }

    pub fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            spacecraft_acc: self.cur.acc,
            spacecraft_altitude: Some(self.cur.pos.y),
        })
    }

    pub fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let dt = 1.0;

        // compute t, mass

        let t = self.cur.t + dt;

        let sc_fuel_mass = self.cur.fuel_mass - self.spec.nominal_mass_flow*control.engine_throttle*dt;

        // self.dv_flight += acc*dt

        // compute torque and angular vel/pos

        let torque = 4.0*control.engine_throttle*self.spec.nominal_thrust*(control.engine_gimbal).sin();

        let sc_mass = self.spec.dry_mass + self.cur.fuel_mass;
        let sc_moment_of_inertia = 0.5 * sc_mass * 2.0_f64.powi(2);  // 1/2*m*r**2 = kg.m**2

        let sc_ang_acc = torque/sc_moment_of_inertia;

        println!("sc_ang_acc {:?} deg", sc_ang_acc*180.0/3.14);

        let sc_ang_vel = self.cur.ang_vel + sc_ang_acc*dt;
        let sc_ang_pos = self.cur.ang_pos + sc_ang_vel*dt;

        // compute thrust and acc/vel/pos

        let sc_mass = self.spec.dry_mass + self.cur.fuel_mass;

        let engine_acc_norm = control.engine_throttle * self.spec.nominal_thrust/sc_mass;
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

        self.cur.t = t;

        self.cur.eng_gimbal = control.engine_gimbal;
        self.cur.eng_throttle = control.engine_throttle;

        self.cur.fuel_mass = sc_fuel_mass;

        self.cur.pos = sc_pos;
        self.cur.vel = sc_vel;
        self.cur.acc = sc_acc;

        self.cur.ang_pos = sc_ang_pos;
        self.cur.ang_vel = sc_ang_vel;

        self.all.push(self.cur);

        println!("{:?}", self.cur);

        Ok(())
    }
}
