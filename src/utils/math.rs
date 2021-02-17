use std::ops::{Add, Sub, AddAssign};

use serde::{Serialize, Deserialize};


#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Vec2<T> {
    pub x: T,   // horizontal axis
    pub y: T,   // vertical axis
}


impl<T: std::ops::Add<Output = T>> Add for Vec2<T> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}


impl<T: std::ops::Sub<Output = T>> Sub for Vec2<T> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}


impl<T: std::ops::AddAssign> AddAssign for Vec2<T> {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}


pub fn sign(val: f64) -> f64 {
    if val < 0.0 {
        -1.0
    } else {
        1.0
    }
}


pub fn saturate<T: std::cmp::PartialOrd>(val: T, min: T, max: T) -> T {
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
