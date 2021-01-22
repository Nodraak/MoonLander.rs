use std::ops::{Add, AddAssign, Mul};


#[derive(Debug)]
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


impl AddAssign for Vec2 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
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


pub fn sign(val: f64) -> f64 {
    if val < 0.0 {
        -1.0
    } else {
        1.0
    }
}


pub fn saturate(val: f64, min: f64, max: f64) -> f64 {
    if val < min {
        min
    } else if max < val {
        max
    } else {
        val
    }
}


#[macro_export]
macro_rules! assert_approx_eq {
    ($a:expr, $b:expr, $eps:expr) => {{
        let (a, b) = (&$a, &$b);
        let eps = $eps;
        assert!(
            (*a - *b).abs() < eps,
            "assertion failed: `(left !== right)` \
             (left: `{:?}`, right: `{:?}`, expect diff: `{:?}`, real diff: `{:?}`)",
            *a,
            *b,
            eps,
            (*a - *b).abs()
        );
    }};
}
