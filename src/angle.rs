use std::{
    f64::consts::PI,
    fmt::Display,
    ops::{Add, AddAssign, Div, Mul, Sub, SubAssign},
};

use serde::{Deserialize, Serialize};

use crate::{
    point::{FloatPoint, GridPoint},
    pt,
};

pub trait Angle: Copy {
    fn bound() -> f64;

    fn degrees(&self) -> Degrees;

    fn radians(&self) -> Radians;

    fn sin(&self) -> f64;

    fn cos(&self) -> f64;

    fn abs(&self) -> Self;

    fn normalize_angle(angle: f64) -> f64 {
        let mut angle = angle;
        let half_bound = Self::bound() / 2.0;
        while angle <= -half_bound {
            angle += Self::bound();
        }
        while angle > half_bound {
            angle -= Self::bound();
        }
        angle
    }

    fn as_f64(&self) -> f64;
}

pub fn angle_distance<A: Angle + Sub<Output = A>>(angle1: A, angle2: A) -> A {
    (angle1 - angle2).abs()
}

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Radians(f64);

impl Angle for Radians {
    fn bound() -> f64 {
        PI * 2.0
    }

    fn degrees(&self) -> Degrees {
        (*self).into()
    }

    fn radians(&self) -> Radians {
        *self
    }

    fn abs(&self) -> Self {
        Self::new(self.0.abs())
    }

    fn as_f64(&self) -> f64 {
        self.0
    }

    fn sin(&self) -> f64 {
        self.as_f64().sin()
    }

    fn cos(&self) -> f64 {
        self.as_f64().cos()
    }
}

impl Display for Radians {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

macro_rules! assign_code {
    ($type:tt) => {
        impl AddAssign for $type {
            fn add_assign(&mut self, rhs: Self) {
                *self = *self + rhs;
            }
        }

        impl SubAssign for $type {
            fn sub_assign(&mut self, rhs: Self) {
                *self = *self - rhs;
            }
        }
    };
}

macro_rules! angle_code {
    ($type:tt) => {
        impl $type {
            pub fn new(angle: f64) -> Self {
                Self(Self::normalize_angle(angle))
            }
        }

        impl From<$type> for f64 {
            fn from(value: $type) -> Self {
                value.0
            }
        }

        impl Add for $type {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                Self::new(self.0 + rhs.0)
            }
        }

        impl Sub for $type {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self::new(self.0 - rhs.0)
            }
        }

        impl Mul<f64> for $type {
            type Output = Self;

            fn mul(self, rhs: f64) -> Self::Output {
                Self::new(self.0 * rhs)
            }
        }

        impl Div<f64> for $type {
            type Output = Self;

            fn div(self, rhs: f64) -> Self::Output {
                Self::new(self.0 / rhs)
            }
        }

        assign_code!($type);
    };
}

angle_code!(Radians);

impl From<FloatPoint> for f64 {
    fn from(value: FloatPoint) -> Self {
        value.iter().map(|n| n.powf(2.0)).sum::<f64>().sqrt()
    }
}

impl From<FloatPoint> for (f64, Degrees) {
    fn from(value: FloatPoint) -> Self {
        let radians: Radians = value.into();
        (value.into(), radians.into())
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Degrees(f64);

impl Angle for Degrees {
    fn bound() -> f64 {
        360.0
    }

    fn degrees(&self) -> Degrees {
        *self
    }

    fn radians(&self) -> Radians {
        (*self).into()
    }

    fn sin(&self) -> f64 {
        self.radians().sin()
    }

    fn cos(&self) -> f64 {
        self.radians().cos()
    }

    fn abs(&self) -> Self {
        Self::new(self.0.abs())
    }

    fn as_f64(&self) -> f64 {
        self.0
    }
}

impl Display for Degrees {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.4}\u{00B0}", self.0)
    }
}

angle_code!(Degrees);

impl From<FloatPoint> for Radians {
    fn from(value: FloatPoint) -> Self {
        Self::new(value[1].atan2(value[0]))
    }
}

impl From<Radians> for Degrees {
    fn from(value: Radians) -> Self {
        Self::new(value.0 * 180.0 / PI)
    }
}

impl From<Degrees> for Radians {
    fn from(value: Degrees) -> Self {
        Self::new(value.0 * PI / 180.0)
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq, Debug)]
pub struct Polar {
    r: f64,
    theta: Radians,
}

impl Polar {
    pub fn new(r: f64, theta: Radians) -> Self {
        Self { r, theta }
    }

    pub fn r(&self) -> f64 {
        self.r
    }

    pub fn theta(&self) -> Radians {
        self.theta
    }
}

impl From<Polar> for FloatPoint {
    fn from(value: Polar) -> Self {
        pt!(value.r * value.theta.cos(), value.r * value.theta.sin())
    }
}

impl From<FloatPoint> for Polar {
    fn from(value: FloatPoint) -> Self {
        Polar {
            r: (value[0].powf(2.0) + value[1].powf(2.0)).sqrt(),
            theta: Radians::new(value[1].atan2(value[0])),
        }
    }
}

impl Add<Polar> for FloatPoint {
    type Output = Self;

    fn add(self, rhs: Polar) -> Self::Output {
        let rhsfp: FloatPoint = rhs.into();
        self + rhsfp
    }
}

impl Add<Polar> for GridPoint {
    type Output = Self;

    fn add(self, rhs: Polar) -> Self::Output {
        let fp = pt!(self[0] as f64, self[1] as f64);
        let sum = fp + rhs;
        pt!(sum[0] as i64, sum[1] as i64)
    }
}

#[cfg(test)]
mod tests {
    use crate::angle::{Angle, Degrees, Radians, angle_distance};

    #[test]
    fn test_distance() {
        for (baseline, angle, distance) in [
            (90.0, 70.0, 20.0),
            (90.0, 110.0, 20.0),
            (-90.0, -70.0, 20.0),
            (-90.0, -110.0, 20.0),
        ] {
            let baseline = Degrees::new(baseline);
            let angle = Degrees::new(angle);
            let distance = Degrees::new(distance);
            let actual = (baseline - angle).abs();
            assert_eq!(actual, distance);
            assert_eq!(angle_distance(baseline, angle), actual);
        }
    }

    #[test]
    fn test_mul_div() {
        for (radians, scalar) in [(2.0, 2.0), (-3.0, 1.5), (5.0, -1.0)] {
            let product = Radians::new(radians * scalar);
            let quotient = Radians::new(radians / scalar);
            let radians = Radians::new(radians);
            assert_eq!(radians * scalar, product);
            assert_eq!(radians / scalar, quotient);
        }
    }
}
