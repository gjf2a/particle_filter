use std::{f64::consts::PI, fmt::Display, ops::{Add, AddAssign, Neg, Sub, SubAssign}};

use crate::point::FloatPoint;

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Radians(f64);

impl Radians {
    pub fn new(angle_radians: f64) -> Self {
        Self(normalize_angle(angle_radians, PI * 2.0))
    }
}

impl From<(f64, Radians)> for FloatPoint {
    fn from(value: (f64, Radians)) -> Self {
        let (r, theta) = value;
        FloatPoint::new([r * theta.0.cos(), r * theta.0.sin()])
    }
}

impl Display for Radians {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl AddAssign for Radians {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Add for Radians {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Neg for Radians {
    type Output = Self;

    fn neg(self) -> Self::Output {
        self + Radians(PI)
    }
}

impl SubAssign for Radians {
    fn sub_assign(&mut self, rhs: Self) {
        *self += -rhs;
    }
}

impl Sub for Radians {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let mut result = rhs;
        result -= rhs;
        result
    }
}

fn normalize_angle(value: f64, bound: f64) -> f64 {
    let mut angle = value;
    let half_bound = bound / 2.0;
    while angle <= -half_bound {
        angle += bound;
    }
    while angle > half_bound {
        angle -= bound;
    }
    angle
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Degrees(f64);

impl Degrees {
    pub fn new(angle_degrees: f64) -> Self {
        Self(normalize_angle(angle_degrees, 360.0))
    }
}

impl AddAssign for Degrees {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Add for Degrees {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Neg for Degrees {
    type Output = Self;

    fn neg(self) -> Self::Output {
        self + Degrees(180.0)
    }
}

impl SubAssign for Degrees {
    fn sub_assign(&mut self, rhs: Self) {
        *self += -rhs;
    }
}

impl Sub for Degrees {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let mut result = rhs;
        result -= rhs;
        result
    }
}

impl Display for Degrees {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}\u{00B0}", self.0)
    }
}

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

impl From<Radians> for f64 {
    fn from(value: Radians) -> Self {
        value.0
    }
}

impl From<Degrees> for f64 {
    fn from(value: Degrees) -> Self {
        value.0
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose {
    pub pos: FloatPoint,
    pub theta: Radians,
}

impl AddAssign for RobotPose {
    fn add_assign(&mut self, rhs: Self) {
        self.pos += rhs.pos;
        self.theta += rhs.theta;
    }
}