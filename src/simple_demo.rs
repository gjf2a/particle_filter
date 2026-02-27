use std::f64::consts::PI;

use crate::nums::{Radians, RobotPose};
use crate::{Angle, BoundingBox, FloatPoint, Noise, ObstacleMap};

// This simple demonstration involves a simulated robot in a circular-fenced
// area. Its sensor determines the forward distance to the fence based on
// its current position and orientation. On each move, a certain amount of noise
// is added to the true position.

pub struct CircleSimulator {
    true_position: RobotPose<Radians>,
    odometry_position: RobotPose<Radians>,
    noise: Noise,
    circle_model: CircleFence,
    script: Script,
}

impl CircleSimulator {
    pub fn new(moves: Vec<Move>, radius: f64, noise: Noise) -> Self {
        Self {
            true_position: RobotPose::default(),
            odometry_position: RobotPose::default(),
            noise,
            circle_model: CircleFence::new(radius),
            script: Script::new(moves),
        }
    }

    pub fn ground_truth_pose(&self) -> RobotPose<Radians> {
        self.true_position
    }

    pub fn odometry_pose(&self) -> RobotPose<Radians> {
        self.odometry_position
    }

    pub fn tick_sensor(&mut self) -> Option<f64> {
        let tick_move = self.script.next().unwrap();
        self.odometry_position = tick_move.advance(self.odometry_position);
        self.true_position = self.noise.noise(tick_move.advance(self.true_position));
        self.circle_model.distance_to_edge(self.true_position)
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Move {
    Forward,
    Left,
    Right,
}

impl Move {
    pub fn advance(&self, pose: RobotPose<Radians>) -> RobotPose<Radians> {
        match self {
            Move::Forward => {
                let movement: FloatPoint = (1.0, pose.theta).into();
                pose + movement
            }
            Move::Left => pose + Radians::new(PI / 4.0),
            Move::Right => pose - Radians::new(PI / 4.0),
        }
    }
}

pub struct Script {
    moves: Vec<Move>,
    current: usize,
}

impl Script {
    pub fn new(moves: Vec<Move>) -> Self {
        Self {
            moves,
            current: 0
        }
    }
}

impl Iterator for Script {
    type Item = Move;

    fn next(&mut self) -> Option<Self::Item> {
        let result = Some(self.moves[self.current]);
        self.current = (self.current + 1) % self.moves.len();
        result
    }
}

#[derive(Clone, PartialEq)]
pub struct CircleFence {
    radius: f64,
}

impl CircleFence {
    pub fn new(radius: f64) -> Self {
        Self {
            radius,
        }
    }

    pub fn distance_to_edge(&self, pose: RobotPose<Radians>) -> Option<f64> {
        let angle_offset: Radians = pose.pos.into();
        // Math from: https://www.perplexity.ai/search/there-is-a-point-inside-a-circ-XYdtqknMRguLF9J9pVEkkQ
        let direction_vector = FloatPoint::new([angle_offset.cos(), angle_offset.sin()]);
        let a = direction_vector.dot(&direction_vector);
        let b = 2.0 * pose.pos.dot(&direction_vector);
        let c = pose.pos.dot(&pose.pos) - self.radius.powf(2.0);
        let discriminant = b.powf(2.0) - 4.0 * a * c;
        if discriminant > 0.0 {
            Some((-b + discriminant.sqrt()) / (2.0 * a))
        } else {
            None
        }
    }

    pub fn contains(&self, p: FloatPoint) -> bool {
        let d: f64 = p.into();
        d <= self.radius
    }
}

#[derive(PartialEq, Clone)]
pub struct DummyMap {
    error: f64,
    map: CircleFence,
    noise: Noise,
}

impl DummyMap {
    pub fn new(radius: f64, noise: Noise) -> Self {
        Self {
            error: 0.0,
            map: CircleFence::new(radius),
            noise
        }
    }
}

impl ObstacleMap for DummyMap {
    type SensorType = f64;

    type ErrorType = f64;

    fn error(&self) -> Self::ErrorType {
        self.error
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        if let Some(distance) = sensor_info {
            if let Some(map_distance) = self.map.distance_to_edge(pose) {
                self.error += (*distance - map_distance).abs();
            } else {
                self.error *= 100.0;
            }
        }
    }

    fn noise(&self, _: Option<&Self::SensorType>) -> Noise {
        self.noise
    }

    fn bounding_box(&self) -> BoundingBox {
        todo!()
    }
}
