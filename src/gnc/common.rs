use crate::utils::math::Vec2;
use crate::spacecraft::{SpacecraftStatic, SpacecraftDynamic};


pub struct Spacecraft {
    spec: SpacecraftStatic,         // fixed properties
    cur: SpacecraftDynamic,         // latest changing properties
    all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new() -> Spacecraft {
        Spacecraft {
            spec: SpacecraftStatic {
                nominal_thrust: 0.0,
                nominal_mass_flow: 1.0, // TODO from conf
                dry_mass: 0.0,  // TODO from conf
            },
            cur: SpacecraftDynamic {
                t: 0.0,
                fuel_mass: 0.0,  // TODO from conf
                pos: Vec2 {x: 0.0, y: 0.0},
                vel: Vec2 {x: 0.0, y: 0.0},
                acc: Vec2 {x: 0.0, y: 0.0},
                ang_pos: 0.0,
                ang_vel: 0.0,
            },
            all: vec![],
        }
    }
}
