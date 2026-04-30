use std::{
    f64::consts::PI,
    fmt::Display,
    ops::{Add, AddAssign, Sub, SubAssign},
};

use serde::{Deserialize, Serialize};

use crate::point::FloatPoint;

pub trait Angle {
    fn bound() -> f64;

    fn degrees(&self) -> Degrees;

    fn radians(&self) -> Radians;

    fn sin(&self) -> f64;

    fn cos(&self) -> f64;

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

    fn sin(&self) -> f64 {
        self.0.sin()
    }

    fn cos(&self) -> f64 {
        self.0.cos()
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

            pub fn abs(&self) -> Self {
                if self.0 < 0.0 {
                    $type::new(-self.0)
                } else {
                    *self
                }
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

        assign_code!($type);
    };
}

angle_code!(Radians);

impl From<(f64, Radians)> for FloatPoint {
    fn from(value: (f64, Radians)) -> Self {
        let (r, theta) = value;
        FloatPoint::new([r * theta.0.cos(), r * theta.0.sin()])
    }
}

impl From<FloatPoint> for f64 {
    fn from(value: FloatPoint) -> Self {
        value.iter().map(|n| n.powf(2.0)).sum::<f64>().sqrt()
    }
}

impl From<FloatPoint> for (f64, Radians) {
    fn from(value: FloatPoint) -> Self {
        (value.into(), value.into())
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

#[cfg(test)]
mod tests {
    use crate::angle::Degrees;

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
        }
    }
}
