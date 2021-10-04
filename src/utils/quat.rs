// You know what? I can not get this to compile, so fuck DRY.

use std::f64::consts::PI;
use std::ops::{Add, AddAssign, Mul};

use serde::{Serialize, Deserialize};
use uom::si::angle::radian;
use uom::si::angular_acceleration::radian_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::time::second;
use uom::si::f64::{Angle, AngularAcceleration, AngularVelocity, Time};

use crate::utils::math::sign;


/// Hamilton convention
/// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
/// https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
/// https://github.com/PX4/PX4-Matrix/blob/master/test/attitude.cpp
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Quaternion<T> {
    w: T,
    x: T,
    y: T,
    z: T,
}

impl PartialEq for Quaternion<f64> {
    fn eq(&self, other: &Self) -> bool {
        (
            expect_float_absolute_eq!(self.w, other.w).is_ok()
            && expect_float_absolute_eq!(self.x, other.x).is_ok()
            && expect_float_absolute_eq!(self.y, other.y).is_ok()
            && expect_float_absolute_eq!(self.z, other.z).is_ok()
        )
    }
}

/// Euler angles
/// https://en.wikipedia.org/wiki/Euler_angles
/// yaw (Z), pitch (Y), roll (X)
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Euler<T> {
    roll: T,
    pitch: T,
    yaw: T,
}

impl PartialEq for Euler<f64> {
    fn eq(&self, other: &Self) -> bool {
        (
            expect_float_absolute_eq!(self.roll, other.roll).is_ok()
            && expect_float_absolute_eq!(self.pitch, other.pitch).is_ok()
            && expect_float_absolute_eq!(self.yaw, other.yaw).is_ok()
        )
    }
}

//
// Euler
//

impl Euler<f64> {
    pub fn to_quaternion(self) -> Quaternion<f64> {
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


impl Euler<Angle> {
    pub fn to_quaternion(self) -> Quaternion<Angle> {
        let qf64 = self.to_f64().to_quaternion();

        Quaternion {
            w: Angle::new::<radian>(qf64.w),
            x: Angle::new::<radian>(qf64.x),
            y: Angle::new::<radian>(qf64.y),
            z: Angle::new::<radian>(qf64.z),
        }
    }

    pub fn to_f64(self) -> Euler<f64> {
        Euler {
            roll: self.roll.get::<radian>(),
            pitch: self.pitch.get::<radian>(),
            yaw: self.yaw.get::<radian>(),
        }
    }
}


impl Euler<AngularVelocity> {
    pub fn to_quaternion(self) -> Quaternion<AngularVelocity> {
        let qf64 = self.to_f64().to_quaternion();

        Quaternion {
            w: AngularVelocity::new::<radian_per_second>(qf64.w),
            x: AngularVelocity::new::<radian_per_second>(qf64.x),
            y: AngularVelocity::new::<radian_per_second>(qf64.y),
            z: AngularVelocity::new::<radian_per_second>(qf64.z),
        }
    }

    pub fn to_f64(self) -> Euler<f64> {
        Euler {
            roll: self.roll.get::<radian_per_second>(),
            pitch: self.pitch.get::<radian_per_second>(),
            yaw: self.yaw.get::<radian_per_second>(),
        }
    }
}


impl Euler<AngularAcceleration> {
    pub fn to_quaternion(self) -> Quaternion<AngularAcceleration> {
        let qf64 = self.to_f64().to_quaternion();

        Quaternion {
            w: AngularAcceleration::new::<radian_per_second_squared>(qf64.w),
            x: AngularAcceleration::new::<radian_per_second_squared>(qf64.x),
            y: AngularAcceleration::new::<radian_per_second_squared>(qf64.y),
            z: AngularAcceleration::new::<radian_per_second_squared>(qf64.z),
        }
    }

    pub fn to_f64(self) -> Euler<f64> {
        Euler {
            roll: self.roll.get::<radian_per_second_squared>(),
            pitch: self.pitch.get::<radian_per_second_squared>(),
            yaw: self.yaw.get::<radian_per_second_squared>(),
        }
    }
}

//
// Quaternion<f64>
//

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

    pub fn normalize(self) -> Quaternion<f64> {
        let norm = (
            self.w.powf(2.0) + self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0)
        ).sqrt();

        self * (1.0/norm)
    }

    // dv/dt += 0.5*self*q
    // => dv += 0.5*self*q*dt
    pub fn integrate(&mut self, q: Quaternion<f64>, dt: Time) {
        let dt = dt.get::<second>();

        (*self) = (*self * q * 0.5 * dt).normalize();
    }
}

impl Add for Quaternion<f64> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Quaternion {
            w: self.w + other.w,
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Mul<Quaternion<f64>> for Quaternion<f64> {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Quaternion {
            // wikipedia?
            w: self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z,
            x: self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y,
            y: self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x,
            z: self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w,

            // cur px4 code?
            // w: self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z,
            // x: self.w*other.x + self.x*other.w - self.y*other.z + self.z*other.y,
            // y: self.w*other.y + self.x*other.z + self.y*other.w - self.z*other.x,
            // z: self.w*other.z - self.x*other.y + self.y*other.x + self.z*other.w,
        }
    }
}

impl Mul<f64> for Quaternion<f64> {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
        Quaternion {
            w: self.w * other,
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}

//
// Quaternion<Angle>
//

impl Quaternion<Angle> {
    pub fn to_euler(self) -> Euler<Angle> {
        let ef64 = self.to_f64().to_euler();

        Euler {
            roll: Angle::new::<radian>(ef64.roll),
            pitch: Angle::new::<radian>(ef64.pitch),
            yaw: Angle::new::<radian>(ef64.yaw),
        }
    }

    pub fn to_f64(self) -> Quaternion<f64> {
        Quaternion {
            w: self.w.get::<radian>(),
            x: self.x.get::<radian>(),
            y: self.y.get::<radian>(),
            z: self.z.get::<radian>(),
        }
    }

    // dp/dt += 0.5*p*vel
    // => dp += 0.5*p*vel*dt
    pub fn integrate(&mut self, vel: Quaternion<AngularVelocity>, dt: Time) {
        let mut self64 = self.to_f64();
        self64.integrate(vel.to_f64(), dt);
        (*self) = Quaternion {
            w: Angle::new::<radian>(self64.w),
            x: Angle::new::<radian>(self64.x),
            y: Angle::new::<radian>(self64.y),
            z: Angle::new::<radian>(self64.z),
        };
    }
}

impl AddAssign for Quaternion<Angle> {
    fn add_assign(&mut self, other: Self) {
        self.w += other.w;
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

//
// Quaternion<AngularVelocity>
//

impl Quaternion<AngularVelocity> {
    pub fn to_euler(self) -> Euler<AngularVelocity> {
        let ef64 =  self.to_f64().to_euler();

        Euler {
            roll: AngularVelocity::new::<radian_per_second>(ef64.roll),
            pitch: AngularVelocity::new::<radian_per_second>(ef64.pitch),
            yaw: AngularVelocity::new::<radian_per_second>(ef64.yaw),
        }
    }

    pub fn to_f64(self) -> Quaternion<f64> {
        Quaternion {
            w: self.w.get::<radian_per_second>(),
            x: self.x.get::<radian_per_second>(),
            y: self.y.get::<radian_per_second>(),
            z: self.z.get::<radian_per_second>(),
        }
    }

    // dv/dt += 0.5*v*acc
    // => dv += 0.5*v*acc*dt
    pub fn integrate(&mut self, acc: Quaternion<AngularAcceleration>, dt: Time) {
        let mut self64 = self.to_f64();
        self64.integrate(acc.to_f64(), dt);
        (*self) = Quaternion {
            w: AngularVelocity::new::<radian_per_second>(self64.w),
            x: AngularVelocity::new::<radian_per_second>(self64.x),
            y: AngularVelocity::new::<radian_per_second>(self64.y),
            z: AngularVelocity::new::<radian_per_second>(self64.z),
        };
    }
}

impl AddAssign for Quaternion<AngularVelocity> {
    fn add_assign(&mut self, other: Self) {
        self.w += other.w;
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

//
// Quaternion<AngularAcceleration>
//

impl Quaternion<AngularAcceleration> {
    pub fn to_euler(self) -> Euler<AngularAcceleration> {
        let ef64 = self.to_f64().to_euler();

        Euler {
            roll: AngularAcceleration::new::<radian_per_second_squared>(ef64.roll),
            pitch: AngularAcceleration::new::<radian_per_second_squared>(ef64.pitch),
            yaw: AngularAcceleration::new::<radian_per_second_squared>(ef64.yaw),
        }
    }

    pub fn to_f64(self) -> Quaternion<f64> {
        Quaternion {
            w: self.w.get::<radian_per_second_squared>(),
            x: self.x.get::<radian_per_second_squared>(),
            y: self.y.get::<radian_per_second_squared>(),
            z: self.z.get::<radian_per_second_squared>(),
        }
    }
}

//
// Tests
//

#[cfg(test)]
mod tests {
    use super::*;

    const euler_check: Euler<f64> = Euler {
        roll: 0.1, pitch: 0.2, yaw: 0.3,
    };
    const q_check: Quaternion<f64> = Quaternion {
        w: 0.98334744, x: 0.0342708, y: 0.10602051, z: 0.14357218,
    };
    const q3: Quaternion<f64> = Quaternion {
        w: 0.0, x: 0.0, y: 0.0, z: 1.0,
    };

    const deg2rad: f64 = PI / 180.0;
    const rad2deg: f64 = 180.0 / PI;

    #[test]
    fn test_quat_normalization() {
        let q0 = Quaternion {w: 1.0, x: 2.0, y: 3.0, z: 4.0};

        let q = q0.normalize();
        assert_eq!(
            q,
            Quaternion {w: 0.18257419, x: 0.36514837, y: 0.54772256, z: 0.73029674},
        );
    }

    #[test]
    fn test_euler_to_quaternion() {
        let q = euler_check.to_quaternion();
        assert_eq!(q, q_check);
    }

    #[test]
    fn test_quaternion_to_euler() {
        let e1 = q_check.to_euler();
        assert_eq!(e1, euler_check);
    }

    #[test]
    fn test_quaternion_product1() {
        let q_prod_check = Quaternion {
            w: 0.93394439, x: 0.0674002, y: 0.20851, z: 0.28236266,
        };
        assert_eq!(q_prod_check, q_check * q_check);
    }

    #[test]
    fn test_quaternion_scalar_multiplication() {
        let scalar = 0.5;
        let q_scalar_mul = Quaternion {w: 1.0, x: 2.0, y: 3.0, z: 4.0};
        let q_scalar_mul_check = Quaternion {
            w: 1.0*scalar, x: 2.0*scalar, y: 3.0*scalar, z: 4.0*scalar,
        };

        let q_scalar_mul_res2 = q_scalar_mul * scalar;
        assert_eq!(q_scalar_mul_check, q_scalar_mul_res2);
    }

    #[test]
    fn test_quaternion_product2() {
        let angle = 2.0*PI;
        let q4: Quaternion<f64> = Quaternion {
            w: (angle/2.0).cos(),
            x: (angle/2.0).sin() * 0.0,
            y: (angle/2.0).sin() * 0.0,
            z: (angle/2.0).sin() * 1.0,
        };
        let q5 = (q4*q4)*q3;

        assert_eq!(q3, q5);
    }

    #[test]
    fn test_quaternion_product3() {
        let angle = PI/2.0;
        let q4: Quaternion<f64> = Quaternion {
            w: (angle/2.0).cos(),
            x: (angle/2.0).sin() * 0.0,
            y: (angle/2.0).sin() * 0.0,
            z: (angle/2.0).sin() * 1.0,
        };
        let q5 = (q4*q4)*q3;
        let q6: Quaternion<f64> = Quaternion {
            w: -1.0, x: 0.0, y: 0.0, z: 0.0,
        };

        assert_eq!(q6, q5);
    }

    #[test]
    // quaternion derivative in frame 1
    fn test_quaternion_derivative1() {
        let mut q1 = Quaternion {w: 0.0, x: 1.0, y: 0.0, z: 0.0};
        let data_q2_check = (Quaternion {w: -0.5, x: 0.0, y: -1.5, z: 1.0}).normalize();

        q1.integrate(Quaternion {w: 0.0, x: 1.0, y: 2.0, z: 3.0}, Time::new::<second>(1.0));

        assert_eq!(q1, data_q2_check);
    }

    #[test]
    // ang_vel.integrate(ang_acc, dt)
    fn test_quaternion_angularAcceleration_integrate() {
        {
            let mut qvel: Quaternion<AngularVelocity> = Euler {
                roll: AngularVelocity::new::<radian_per_second>(0.1),
                pitch: AngularVelocity::new::<radian_per_second>(0.0),
                yaw: AngularVelocity::new::<radian_per_second>(0.0),
            }.to_quaternion();
            let qacc: Quaternion<AngularAcceleration> = Euler {
                roll: AngularAcceleration::new::<radian_per_second_squared>(0.1),
                pitch: AngularAcceleration::new::<radian_per_second_squared>(0.0),
                yaw: AngularAcceleration::new::<radian_per_second_squared>(0.0),
            }.to_quaternion();
            let dt: Time = Time::new::<second>(1.0);

            qvel.integrate(qacc, dt);
            let nvel = qvel.to_f64();

            let e = Euler {roll: 0.2, pitch: 0.0, yaw: 0.0};

            assert_eq!(nvel, e.to_quaternion());
            assert_eq!(nvel.to_euler(), e);
        }
    }

    #[test]
    // ang_pos.integrate(ang_vel, dt)
    fn test_quaternion_angularVelocity_integrate() {
        {
            let mut qpos: Quaternion<Angle> = Euler {
                roll: Angle::new::<radian>(0.1),
                pitch: Angle::new::<radian>(0.0),
                yaw: Angle::new::<radian>(0.0),
            }.to_quaternion();
            let qvel: Quaternion<AngularVelocity> = Euler {
                roll: AngularVelocity::new::<radian_per_second>(0.1),
                pitch: AngularVelocity::new::<radian_per_second>(0.0),
                yaw: AngularVelocity::new::<radian_per_second>(0.0),
            }.to_quaternion();
            let dt: Time = Time::new::<second>(1.0);

            qpos.integrate(qvel, dt);
            let npos = qpos.to_f64();

            let e = Euler {roll: 0.2, pitch: 0.0, yaw: 0.0};

            assert_eq!(npos, e.to_quaternion());
            assert_eq!(npos.to_euler(), e);
        }
    }
}
