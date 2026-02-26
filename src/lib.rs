pub mod nums;
pub mod point;
pub mod stats;

use std::{cmp::Ordering, fmt::Debug, iter::repeat_n};

pub use nums::*;
pub use point::*;

use hash_histogram::HashHistogram;
use rand::{RngExt, rng};
use rand_distr::{Distribution, Normal};

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct Noise {
    pub stdev_x_y: f64,
    pub stdev_angle: Degrees,
}

impl Noise {
    fn noise(&self, pose: RobotPose<Radians>) -> RobotPose<Radians> {
        let mut rng = rand::rng();
        let x_y_gaussian = Normal::new(0.0, self.stdev_x_y).unwrap();
        let theta_gaussian = Normal::new(0.0, self.stdev_angle.into()).unwrap();
        let x_y_noise =
            FloatPoint::new([x_y_gaussian.sample(&mut rng), x_y_gaussian.sample(&mut rng)]);
        let theta_noise = Degrees::new(theta_gaussian.sample(&mut rng));
        RobotPose {
            pos: (pose.pos + x_y_noise),
            theta: pose.theta + theta_noise.into(),
        }
    }
}

pub trait ObstacleMap: Clone + PartialEq {
    type SensorType;
    type ErrorType : Copy + Clone + PartialOrd + PartialEq + Debug + Default;

    fn error(&self) -> Self::ErrorType;
    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>);
    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise;
    fn bounding_box(&self) -> BoundingBox;
}

#[derive(Clone, Debug, PartialEq, Default)]
pub struct Particle<M: ObstacleMap> {
    estimate: PoseEstimate,
    map: M,
    parent: Option<usize>,
    error: M::ErrorType,
}

impl<M: ObstacleMap> Particle<M> {
    pub fn estimated_pose(&self) -> RobotPose<Radians> {
        self.estimate.into()
    }

    pub fn map(&self) -> &M {
        &self.map
    }

    pub fn parent_index(&self) -> Option<usize> {
        self.parent
    }

    pub fn error(&self) -> M::ErrorType {
        self.error
    }

    fn new(starting_map: &M) -> Self {
        Self {
            estimate: PoseEstimate::default(),
            map: starting_map.clone(),
            parent: None,
            error: M::ErrorType::default(),
        }
    }

    fn sensor_update(&mut self, sensor_info: Option<&M::SensorType>) {
        self.estimate.add_noise(&self.map, sensor_info);
        self.map.sensor_update(self.estimated_pose(), sensor_info);
        self.error = self.map.error();
    }
}

impl<M: ObstacleMap> PartialOrd for Particle<M> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.error.partial_cmp(&other.error).map(|c| c.reverse())
    }
}

#[derive(Clone, Debug)]
pub struct ParticleFilter<M: ObstacleMap> {
    aliases: Vec<usize>,
    particles: Vec<Particle<M>>,
    best_particle: Particle<M>,
}

impl<M: ObstacleMap> ParticleFilter<M> {
    pub fn new(num_particles: usize, starting_map: &M) -> Self {
        let mut aliases = Vec::with_capacity(num_particles * (num_particles + 1) / 2);
        for i in 0..num_particles {
            for _ in 0..=i {
                aliases.push(i);
            }
        }
        let particles = repeat_n(Particle::new(starting_map), num_particles).collect();
        Self {
            aliases,
            particles,
            best_particle: Particle::new(starting_map),
        }
    }

    pub fn particles(&self) -> impl Iterator<Item = &Particle<M>> {
        self.particles.iter()
    }

    pub fn current_best(&self) -> Particle<M> {
        self.best_particle.clone()
    }

    pub fn iterate(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        sensor_info: Option<&M::SensorType>,
    ) {
        self.update_all(new_raw_pose, sensor_info);
        self.resample();
    }

    fn update_all(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        sensor_info: Option<&M::SensorType>,
    ) {
        for particle in self.particles.iter_mut() {
            if let Some(raw_pose) = new_raw_pose {
                particle.estimate.updated_raw_pose(raw_pose);
            }
            particle.sensor_update(sensor_info);
        }
    }

    fn resample(&mut self) {
        self.particles
            .sort_unstable_by(|p1, p2| p1.partial_cmp(p2).unwrap_or(Ordering::Equal));
        self.best_particle = self.particles.last().cloned().unwrap();
        let mut new_particles = vec![];
        let mut rng = rng();
        for _ in 0..self.particles.len() {
            let choice = self.aliases[rng.random_range(0..self.aliases.len())];
            new_particles.push(self.particles[choice].clone());
        }
        std::mem::swap(&mut new_particles, &mut self.particles);
    }
}

pub fn invert_errors(errors: &HashHistogram<usize, f64>) -> HashHistogram<usize, f64> {
    let total = errors.total_count() + errors.len() as f64;
    errors
        .iter()
        .map(|(key, weight)| (*key, total - *weight))
        .collect()
}

#[derive(Default, Clone, Copy, Debug)]
pub struct BoundingBox {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

impl BoundingBox {
    pub fn observe(&mut self, x: f64, y: f64) {
        if self.min_x > x {
            self.min_x = x;
        }
        if self.max_x < x {
            self.max_x = x;
        }
        if self.min_y > y {
            self.min_y = y;
        }
        if self.max_y < y {
            self.max_y = y;
        }
    }
}

impl FromIterator<FloatPoint> for BoundingBox {
    fn from_iter<T: IntoIterator<Item = FloatPoint>>(iter: T) -> Self {
        let mut result = Self::default();
        for point in iter {
            result.observe(point[0], point[1]);
        }
        result
    }
}

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct PoseEstimate {
    last_raw: Option<RobotPose<Radians>>,
    current_estimate: RobotPose<Radians>,
}

impl From<PoseEstimate> for RobotPose<Radians> {
    fn from(value: PoseEstimate) -> Self {
        value.current_estimate
    }
}

impl PoseEstimate {
    pub fn updated_raw_pose(&mut self, raw_pose: RobotPose<Radians>) {
        match self.last_raw {
            None => {
                self.current_estimate = raw_pose;
            }
            Some(last_raw) => {
                self.current_estimate += raw_pose - last_raw;
            }
        }
        self.last_raw = Some(raw_pose);
    }

    pub fn add_noise<M: ObstacleMap>(&mut self, map: &M, sensor_info: Option<&M::SensorType>) {
        self.current_estimate = map.noise(sensor_info).noise(self.current_estimate)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Degrees, FloatPoint, PoseEstimate, Radians, RobotPose};

    #[test]
    fn test_current_estimated_pose() {
        let mut estimate = PoseEstimate::default();
        for (x, y, theta) in [
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (1.0, 1.0, 90.0),
            (1.0, 2.0, 90.0),
        ] {
            let pose = RobotPose {
                pos: FloatPoint::new([x, y]),
                theta: Degrees::new(theta).into(),
            };
            estimate.updated_raw_pose(pose);
            let estimated: RobotPose<Radians> = estimate.into();
            assert_eq!(pose, estimated);
        }
    }
}
