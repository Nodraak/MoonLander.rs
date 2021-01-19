use crate::utils::math::Vec2;

/// fixed properties
#[derive(Clone, Copy)]
pub struct SpacecraftStatic {
    pub nominal_thrust: f64,            // unit: N
    pub nominal_mass_flow: f64,         // unit: kg/s

    pub dry_mass: f64,                  // unit: kg
}

/// changing properties
#[derive(Clone, Copy)]
pub struct SpacecraftDynamic {
    pub t: f64,                         // unit: sec - time since beginning of simulation

    pub fuel_mass: f64,                 // unit: kg

    pub pos: Vec2,                      // unit: m
    pub vel: Vec2,                      // unit: m/s
    pub acc: Vec2,                      // unit: m/s**2

    pub ang_pos: f64,                   // unit: rad
    pub ang_vel: f64,                   // unit: rad/s
}

// TODO constructors, to be used by gnc and sim
