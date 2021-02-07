use std::f64::consts::E;
use serde::{Serialize, Deserialize};


/// Orbited body
///
/// Yaml serialized equivalents:
///
/// # Moon
/// body:
///     radius: 1737.4e3
///     mass: 7.342e22
///     mu: 4.9048695e12
///     atm_r0: 0.0
///     atm_p0: 0.0
///     atm_a: 1.0
///
/// # Earth
/// body:
///     radius: 6371.0e3
///     mass: 5.97237e24
///     mu: 3.986004418e14
///     atm_r0: 1.225
///     atm_p0: 101_325.0
///     atm_a: 8420.0
///
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Body {
    pub radius: f64,                // m - mean radius
    pub mass: f64,                  // kg
    pub mu: f64,                    // m**3/s**2 - standard gravitational parameter

    pub atm_r0: f64,                // kg/m3 - Air density at sea level
    pub atm_p0: f64,                // N/m2 - Air pressure at sea level
    pub atm_a: f64,                 // m - Effective height of atmosphere
}


impl Body {
    pub fn gravity(&self, altitude: f64) -> f64 {
        self.mu/(self.radius+altitude).powi(2)
    }

    pub fn centrifugal(&self, vel: f64, altitude: f64) -> f64 {
        vel.powi(2)/(self.radius+altitude)
    }

    /// Inputs:
    ///     h: altitude: m
    /// Ouputs:
    ///     rho: density: Kg/m**3
    /// Source: http://scipp.ucsc.edu/outreach/balloon/glost/environment3.html
    pub fn atmosphere_density(&self, altitude: f64) -> f64 {
        let _p = self.atm_p0 * E.powf(-altitude/self.atm_a);  // presure
        let r = self.atm_r0 * E.powf(-altitude/self.atm_a);  // density
        r
    }

    pub fn orbital_velocity(&self, altitude: f64) -> f64 {
        (self.mu/(self.radius+altitude)).powf(0.5)
    }
}
