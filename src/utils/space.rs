
const G0: f64 = 9.806;

const MOON_RADIUS: f64 = 1737.4 * 1e3;      // m
const MOON_MASS: f64 = 7.342 * 1e22;        // kg
const MOON_MU: f64 = 4.9048695 * 1e12;      // m**3/s**2

const MOON_ORBITAL_15000: f64 = 1673.004;   // m/s - (MOON_MU/(MOON_RADIUS+15_000.0)).powf(0.5)

// TODO

// def dv(isp, m_dry, m_wet):
//     return isp*G0*log(m_wet/m_dry)

// def orbital_speed(mu, a, r):
//     return sqrt(mu*(2/r - 1/a))


// def moon_orbital_speed(a, alt):
//     return orbital_speed(MOON_MU, a, MOON_RADIUS+alt)

pub fn moon_gravity(altitude: f64) -> f64 {
    MOON_MU/(MOON_RADIUS+altitude).powi(2)
}

pub fn moon_centrifugal(vel: f64, altitude: f64) -> f64 {
    vel.powi(2)/(MOON_RADIUS+altitude)
}
