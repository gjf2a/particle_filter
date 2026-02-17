use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};

use crate::point::FloatPoint;


#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default)]
pub struct Degrees(f64);

impl Degrees {
    pub fn new(angle: f64) -> Self {
        let mut angle = angle;
        while angle <= -180.0 {
            angle += 360.0;
        }
        while angle > 180.0 {
            angle -= 360.0;
        }
        Self(angle)
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