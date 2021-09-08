// You know what? I can not get this to compile, so fuck DRY.

use std::f64::consts::PI;
use std::ops::{Add, Mul};

use serde::{Serialize, Deserialize};
use uom::si::angle::radian;
use uom::si::angular_acceleration::radian_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Angle, AngularAcceleration, AngularVelocity, Time};

use crate::utils::math::sign;


/// JPL convention
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Quaternion<T> {
    w: T,
    x: T,
    y: T,
    z: T,
}

/// yaw (Z), pitch (Y), roll (X)
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Euler<T> {
    roll: T,
    pitch: T,
    yaw: T,
}

//
// Euler
//

impl Euler<f64> {
    pub fn to_quaternion(self) -> Quaternion<f64> {
        // Abbreviations for the various angular functions
        let cy = (self.yaw * 0.5).cos();
        let sy = (self.yaw * 0.5).sin();
        let cp = (self.pitch * 0.5).cos();
        let sp = (self.pitch * 0.5).sin();
        let cr = (self.roll * 0.5).cos();
        let sr = (self.roll * 0.5).sin();

        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}


impl Quaternion<f64> {
    pub fn to_euler(self) -> Euler<f64> {
        // roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if (sinp.abs() >= 1.0) {
            sign(sinp) * PI/2.0  // use 90 degrees if out of range
        } else {
            sinp.asin()
        };

        // yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        // ret
        return Euler {
            roll: roll,
            pitch: pitch,
            yaw: yaw,
        }
    }
}


impl Euler<Angle> {
    pub fn to_quaternion(self) -> Quaternion<Angle> {
        let qf64 = Euler {
            roll: self.roll.get::<radian>(),
            pitch: self.pitch.get::<radian>(),
            yaw: self.yaw.get::<radian>(),
        }.to_quaternion();

        Quaternion {
            w: Angle::new::<radian>(qf64.w),
            x: Angle::new::<radian>(qf64.x),
            y: Angle::new::<radian>(qf64.y),
            z: Angle::new::<radian>(qf64.z),
        }
    }
}


impl Euler<AngularVelocity> {
    pub fn to_quaternion(self) -> Quaternion<AngularVelocity> {
        let qf64 = Euler {
            roll: self.roll.get::<radian_per_second>(),
            pitch: self.pitch.get::<radian_per_second>(),
            yaw: self.yaw.get::<radian_per_second>(),
        }.to_quaternion();

        Quaternion {
            w: AngularVelocity::new::<radian_per_second>(qf64.w),
            x: AngularVelocity::new::<radian_per_second>(qf64.x),
            y: AngularVelocity::new::<radian_per_second>(qf64.y),
            z: AngularVelocity::new::<radian_per_second>(qf64.z),
        }
    }
}


impl Euler<AngularAcceleration> {
    pub fn to_quaternion(self) -> Quaternion<AngularAcceleration> {
        let qf64 = Euler {
            roll: self.roll.get::<radian_per_second_squared>(),
            pitch: self.pitch.get::<radian_per_second_squared>(),
            yaw: self.yaw.get::<radian_per_second_squared>(),
        }.to_quaternion();

        Quaternion {
            w: AngularAcceleration::new::<radian_per_second_squared>(qf64.w),
            x: AngularAcceleration::new::<radian_per_second_squared>(qf64.x),
            y: AngularAcceleration::new::<radian_per_second_squared>(qf64.y),
            z: AngularAcceleration::new::<radian_per_second_squared>(qf64.z),
        }
    }
}

//
// Quaternion<Angle>
//

impl Quaternion<Angle> {
    pub fn to_euler(self) -> Euler<Angle> {
        let ef64 = Quaternion {
            w: self.w.get::<radian>(),
            x: self.x.get::<radian>(),
            y: self.y.get::<radian>(),
            z: self.z.get::<radian>(),
        }.to_euler();

        Euler {
            roll: Angle::new::<radian>(ef64.roll),
            pitch: Angle::new::<radian>(ef64.pitch),
            yaw: Angle::new::<radian>(ef64.yaw),
        }
    }
}

//
// Quaternion<AngularVelocity>
//

impl Quaternion<AngularVelocity> {
    pub fn to_euler(self) -> Euler<AngularVelocity> {
        let ef64 = Quaternion {
            w: self.w.get::<radian_per_second>(),
            x: self.x.get::<radian_per_second>(),
            y: self.y.get::<radian_per_second>(),
            z: self.z.get::<radian_per_second>(),
        }.to_euler();

        Euler {
            roll: AngularVelocity::new::<radian_per_second>(ef64.roll),
            pitch: AngularVelocity::new::<radian_per_second>(ef64.pitch),
            yaw: AngularVelocity::new::<radian_per_second>(ef64.yaw),
        }
    }

    pub fn integrate(
        self, acc: Quaternion<AngularAcceleration>, dt: Time
    ) -> Quaternion<AngularVelocity> {
        // integrate
        let delta = acc * dt;

        // add the two rotations
        self + delta
    }
}

// Quaternion<AngularVelocity> + Quaternion<AngularVelocity> = Quaternion<AngularVelocity>
impl Add for Quaternion<AngularVelocity> {
    type Output = Self;

    // => dq = 0.5*q*v * dt
    fn add(self, other: Self) -> Self {
        Self {
            w: AngularVelocity::new::<radian_per_second>(1.0),
            x: AngularVelocity::new::<radian_per_second>(1.0),
            y: AngularVelocity::new::<radian_per_second>(1.0),
            z: AngularVelocity::new::<radian_per_second>(1.0),
        }
    }
}

// Quaternion<AngularVelocity> * Time = Quaternion<Angle>
impl Mul<Time> for Quaternion<AngularVelocity> {
    type Output = Quaternion<Angle>;

    fn mul(self, other: Time) -> Quaternion<Angle> {
        Quaternion {
            w: Angle::new::<radian>(1.0),
            x: Angle::new::<radian>(1.0),
            y: Angle::new::<radian>(1.0),
            z: Angle::new::<radian>(1.0),
        }
    }
}

//
// Quaternion<AngularAcceleration>
//

impl Quaternion<AngularAcceleration> {
    pub fn to_euler(self) -> Euler<AngularAcceleration> {
        let ef64 = Quaternion {
            w: self.w.get::<radian_per_second_squared>(),
            x: self.x.get::<radian_per_second_squared>(),
            y: self.y.get::<radian_per_second_squared>(),
            z: self.z.get::<radian_per_second_squared>(),
        }.to_euler();

        Euler {
            roll: AngularAcceleration::new::<radian_per_second_squared>(ef64.roll),
            pitch: AngularAcceleration::new::<radian_per_second_squared>(ef64.pitch),
            yaw: AngularAcceleration::new::<radian_per_second_squared>(ef64.yaw),
        }
    }
}


// Quaternion<AngularAcceleration> * Time = Quaternion<AngularVelocity>
impl Mul<Time> for Quaternion<AngularAcceleration> {
    type Output = Quaternion<AngularVelocity>;

    // => dq = 0.5*q*v * dt
    fn mul(self, other: Time) -> Quaternion<AngularVelocity> {
        Quaternion {
            w: AngularVelocity::new::<radian_per_second>(1.0),
            x: AngularVelocity::new::<radian_per_second>(1.0),
            y: AngularVelocity::new::<radian_per_second>(1.0),
            z: AngularVelocity::new::<radian_per_second>(1.0),
        }
    }
}

//
// Tests
//

// TODO: tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_euler_f64_to_quaternion() {
        let qf64 = Euler {
            roll: 0.5,
            pitch: 0.5,
            yaw: 0.5,
        }.to_quaternion();

        assert_eq!(qf64.w, 0.5);
        assert_eq!(qf64.x, 0.5);
        assert_eq!(qf64.y, 0.5);
        assert_eq!(qf64.z, 0.5);
    }
}
