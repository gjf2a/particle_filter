use std::{f64::consts::PI, fmt::Display, ops::{Add, AddAssign, Neg, Sub, SubAssign}};

use crate::point::FloatPoint;


#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Degrees(f64);

impl Degrees {
    pub fn new(angle_degrees: f64) -> Self {
        let mut angle = angle_degrees;
        while angle <= -180.0 {
            angle += 360.0;
        }
        while angle > 180.0 {
            angle -= 360.0;
        }
        Self(angle)
    }

    pub fn from_radians(angle_radians: f64) -> Self {
        Self::new(angle_radians * 180.0 / PI)
    }

    pub fn point_from(&self, r: f64) -> FloatPoint {
        FloatPoint::new([r * self.0.cos(), r * self.0.sin()])
    }

    pub fn radians(&self) -> f64 {
        self.0 * PI / 180.0
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

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose {
    pub pos: FloatPoint,
    pub theta: Degrees,
}

impl AddAssign for RobotPose {
    fn add_assign(&mut self, rhs: Self) {
        self.pos += rhs.pos;
        self.theta += rhs.theta;
    }
}