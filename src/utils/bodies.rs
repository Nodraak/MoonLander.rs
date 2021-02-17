use std::f64::consts::E;
use serde::{Serialize, Deserialize};
use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::length::meter;
use uom::si::ratio::ratio;
use crate::squared;


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
    pub radius: Length,             // mean radius
    pub mass: Mass,
    // TODO use uom
    pub mu: f64,                    // m**3/s**2 - standard gravitational parameter
                                    //    Note: uom is not used because unit is not supported

    pub atm_r0: MassDensity,        // Air density at sea level
    pub atm_p0: Pressure,           // Air pressure at sea level
    pub atm_a: Length,              // Effective height of atmosphere
}


impl Body {
    pub fn gravity(&self, altitude: Length) -> Acceleration {
        let ra: f64 = (self.radius+altitude).get::<meter>();
        Acceleration::new::<meter_per_second_squared>(self.mu/ra.powi(2))
    }

    pub fn centrifugal(&self, vel: Velocity, altitude: Length) -> Acceleration {
        squared!(vel)/(self.radius+altitude)
    }

    /// Inputs:
    ///     h: altitude: m
    /// Ouputs:
    ///     rho: density: Kg/m**3
    /// Source: http://scipp.ucsc.edu/outreach/balloon/glost/environment3.html
    pub fn atmosphere_density(&self, altitude: Length) -> MassDensity {
        let exp: f64 = (-altitude/self.atm_a).get::<ratio>();
        let _p = self.atm_p0 * E.powf(exp);  // presure
        let r = self.atm_r0 * E.powf(exp);  // density
        r
    }
}
