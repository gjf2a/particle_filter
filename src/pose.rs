use std::{
    fmt::Display,
    ops::{Add, AddAssign, Sub, SubAssign},
};

use serde::{Deserialize, Serialize};

use crate::{
    angle::{Angle, Polar, Radians, angle_distance}, point::{BoundingBox, FloatPoint},
};

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose<A: Angle> {
    pub pos: FloatPoint,
    pub theta: A,
}

impl<A: Angle + Display + Sub<Output = A>> RobotPose<A> {
    pub fn distance(&self, other: &RobotPose<A>, m_per_radian: f64) -> f64 {
        self.pos.euclidean_distance(other.pos)
            + m_per_radian * angle_distance(self.theta, other.theta).as_f64()
    }

    pub fn bounding_box(poses: &Vec<Self>) -> BoundingBox<f64> {
        assert!(poses.len() > 0);
        let mut result = BoundingBox::new(poses[0].pos, poses[0].pos);
        for i in 1..poses.len() {
            result.observe(&poses[i].pos); 
        }
        result
    }
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

impl Add<Polar> for RobotPose<Radians> {
    type Output = FloatPoint;

    fn add(self, rhs: Polar) -> Self::Output {
        let heading = self.theta + rhs.theta();
        self.pos + Polar::new(rhs.r(), heading)
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
