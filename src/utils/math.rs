use std::ops::{Add, Mul};


#[derive(Clone, Copy)]
pub struct Vec2 {
    pub x: f64,  // horizontal axis
    pub y: f64,  // vertical axis
}


impl Vec2 {
    pub fn new_polar(norm: f64, angle: f64) -> Vec2 {
        Vec2 {
            x: norm * angle.cos(),
            y: norm * angle.sin(),
        }
    }

}


impl Add for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}


impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
        Self {
            x: self.x + other,
            y: self.y + other,
        }
    }
}
