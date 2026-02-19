use std::{
    f64::consts::PI,
    fmt::Display,
    ops::{Add, AddAssign, Sub, SubAssign},
};

use crate::point::FloatPoint;

pub trait Angle {
    fn bound() -> f64;

    fn degrees(&self) -> f64;

    fn radians(&self) -> f64;

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

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Radians(f64);

impl Angle for Radians {
    fn bound() -> f64 {
        PI * 2.0
    }

    fn degrees(&self) -> f64 {
        let deg: Degrees = (*self).into();
        deg.into()
    }

    fn radians(&self) -> f64 {
        (*self).into()
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

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Degrees(f64);

impl Angle for Degrees {
    fn bound() -> f64 {
        360.0
    }

    fn degrees(&self) -> f64 {
        (*self).into()
    }

    fn radians(&self) -> f64 {
        let deg: Radians = (*self).into();
        deg.into()
    }
}

impl Display for Degrees {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.2}\u{00B0}", self.0)
    }
}

angle_code!(Degrees);

impl From<Radians> for Degrees {
    fn from(value: Radians) -> Self {
        Degrees::new(value.0 * 180.0 / PI)
    }
}

impl From<Degrees> for Radians {
    fn from(value: Degrees) -> Self {
        Radians::new(value.0 * PI / 180.0)
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose<A: Angle> {
    pub pos: FloatPoint,
    pub theta: A,
}

impl<A: Angle + Display> Display for RobotPose<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.3}, {:.3});{}", self.pos[0], self.pos[1], self.theta)
    }
}

impl<A: Angle + Add<Output=A>> Add for RobotPose<A> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            pos: self.pos + rhs.pos,
            theta: self.theta + rhs.theta,
        }
    }
}

impl<A: Angle + Sub<Output=A>> Sub for RobotPose<A> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            pos: self.pos - rhs.pos,
            theta: self.theta - rhs.theta,
        }
    }
}

impl<A: Angle + Add<Output=A> + Copy> AddAssign for RobotPose<A> {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl<A: Angle + Sub<Output=A> + Copy> SubAssign for RobotPose<A> {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}