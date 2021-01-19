use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::spacecraft::{SpacecraftStatic, SpacecraftDynamic};
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


#[derive(Clone, Copy)]
pub struct SimValues {
    // changing properties
    sc: SpacecraftDynamic,

    // control
    engine_gimbal: f64,             // unit: rad
    engine_throttle: f64,           // unit: 0-1
}


pub struct Sim {
    spec: SpacecraftStatic,         // fixed properties
    cur: SimValues,                 // latest changing properties
    all: Vec<SimValues>,            // all changing properties
}


impl Sim {
    pub fn new() -> Sim {
        Sim {
            spec: SpacecraftStatic {
                nominal_thrust: 0.0,
                nominal_mass_flow: 1.0, // TODO from conf
                dry_mass: 0.0,  // TODO from conf
            },
            cur: SimValues {
                sc: SpacecraftDynamic {
                    t: 0.0,
                    fuel_mass: 0.0,  // TODO from conf
                    pos: Vec2 {x: 0.0, y: 0.0},
                    vel: Vec2 {x: 0.0, y: 0.0},
                    acc: Vec2 {x: 0.0, y: 0.0},
                    ang_pos: 0.0,
                    ang_vel: 0.0,
                },
                engine_gimbal: 0.0,
                engine_throttle: 0.0,
            },
            all: vec![],
        }
    }

    pub fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            spacecraft_acc: self.cur.sc.acc,
            spacecraft_altitude: Some(self.cur.sc.pos.y),
        })
    }

    pub fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let dt = 1.0;

        // compute everything

        // TODO first compute torque and update sc_angle, then compute sc_acc

        let sc_mass = self.spec.dry_mass + self.cur.sc.fuel_mass;

        let engine_acc_norm = control.engine_throttle * self.spec.nominal_thrust/sc_mass;
        let engine_acc = Vec2::new_polar(engine_acc_norm, self.cur.sc.ang_pos);
        let gravity_acc = Vec2 {
            x: 0.0,
            y:
                - moon_gravity(self.cur.sc.pos.y)
                + moon_centrifugal(self.cur.sc.vel.x, self.cur.sc.pos.y),
        };
        let sc_acc = engine_acc + gravity_acc;
        // self.g = self.acc_y/G0

        let sc_vel = self.cur.sc.vel + sc_acc*dt;
        let sc_pos = self.cur.sc.pos + sc_vel*dt;

        let sc_fuel_mass = self.cur.sc.fuel_mass - self.spec.nominal_mass_flow*control.engine_throttle*dt;

        // self.dv_flight += acc*dt

        let t = self.cur.sc.t + dt;

        // save everything

        self.cur.sc.t = t;

        self.cur.sc.fuel_mass = sc_fuel_mass;
        self.cur.sc.pos = sc_pos;
        self.cur.sc.vel = sc_vel;
        self.cur.sc.acc = sc_acc;
        // self.cur.sc_ang_pos = sc_ang_pos; TODO
        // self.cur.sc_ang_vel = sc_ang_vel; TODO

        self.cur.engine_gimbal = control.engine_gimbal;
        self.cur.engine_throttle = control.engine_throttle;

        self.all.push(self.cur);

        Ok(())
    }
}
