use std::{
    fmt::Display,
    ops::{Add, AddAssign, Sub, SubAssign},
};

use serde::{Deserialize, Serialize};

use crate::{angle::Angle, point::FloatPoint};

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose<A: Angle> {
    pub pos: FloatPoint,
    pub theta: A,
}

impl<A: Angle + Display> Display for RobotPose<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let degrees = self.theta.degrees();
        write!(f, "({:.3}, {:.3});{degrees}", self.pos[0], self.pos[1])
    }
}

impl<A: Angle + Add<Output = A>> Add for RobotPose<A> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            pos: self.pos + rhs.pos,
            theta: self.theta + rhs.theta,
        }
    }
}

impl<A: Angle + Add<Output = A>> Add<FloatPoint> for RobotPose<A> {
    type Output = Self;

    fn add(self, rhs: FloatPoint) -> Self::Output {
        Self {
            pos: self.pos + rhs,
            theta: self.theta,
        }
    }
}

impl<A: Angle + Add<Output = A>> Add<A> for RobotPose<A> {
    type Output = Self;

    fn add(self, rhs: A) -> Self::Output {
        Self {
            pos: self.pos,
            theta: self.theta + rhs,
        }
    }
}

impl<A: Angle + Sub<Output = A>> Sub for RobotPose<A> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            pos: self.pos - rhs.pos,
            theta: self.theta - rhs.theta,
        }
    }
}

impl<A: Angle + Sub<Output = A>> Sub<A> for RobotPose<A> {
    type Output = Self;

    fn sub(self, rhs: A) -> Self::Output {
        Self {
            pos: self.pos,
            theta: self.theta - rhs,
        }
    }
}

impl<A: Angle + Add<Output = A> + Copy> AddAssign for RobotPose<A> {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl<A: Angle + Sub<Output = A> + Copy> SubAssign for RobotPose<A> {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}
