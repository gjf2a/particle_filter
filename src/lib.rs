pub mod nums;
pub mod point;

pub use nums::*;
pub use point::*;

use hash_histogram::HashHistogram;
use rand_distr::{Distribution, Normal};

pub trait ObstacleMap: Clone + Default {
    type SensorType;

    fn error(&mut self, estimated_pose: &PoseEstimate) -> f64;
    fn sensor_update(&mut self, estimated_pose: &PoseEstimate, sensor_info: Option<&Self::SensorType>);

    fn mean_stdev(&self, sensor_info: Option<&Self::SensorType>) -> (f64, Degrees);

    fn noise(&self, pose: RobotPose, sensor_info: Option<&Self::SensorType>) -> RobotPose {
        let mut rng = rand::rng();
        let (stdev_x_y, stdev_theta) = self.mean_stdev(sensor_info);
        let x_y_gaussian = Normal::new(0.0, stdev_x_y).unwrap();
        let theta_gaussian = Normal::new(0.0, stdev_theta.into()).unwrap();
        let x_y_noise =
            FloatPoint::new([x_y_gaussian.sample(&mut rng), x_y_gaussian.sample(&mut rng)]);
        let theta_noise = Degrees::new(theta_gaussian.sample(&mut rng));
        RobotPose {
            pos: (pose.pos + x_y_noise),
            theta: pose.theta + theta_noise.into(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ParticleFilter<M: ObstacleMap> {
    particles: Vec<(PoseEstimate, M)>,
    best_particle: (RobotPose, M),
}

impl<M: ObstacleMap> ParticleFilter<M> {
    pub fn new(num_particles: usize) -> Self {
        Self {
            particles: std::iter::repeat((PoseEstimate::default(), M::default()))
                .take(num_particles)
                .collect(),
            best_particle: (RobotPose::default(), M::default()),
        }
    }

    pub fn current_best(&self) -> (RobotPose, M) {
        self.best_particle.clone()
    }

    pub fn iterate(
        &mut self,
        new_raw_pose: Option<RobotPose>,
        sensor_info: Option<&M::SensorType>,
    ) {
        self.resample();
        self.update_all(new_raw_pose, sensor_info);
    }

    fn update_all(&mut self, new_raw_pose: Option<RobotPose>, sensor_info: Option<&M::SensorType>) {
        for (pose, particle) in self.particles.iter_mut() {
            particle.sensor_update(pose, sensor_info);
            if let Some(raw_pose) = new_raw_pose {
                pose.updated_raw_pose(raw_pose);
            }
            pose.add_noise(particle, sensor_info);
        }
    }

    fn resample(&mut self) {
        let errors: HashHistogram<usize, f64> = self
            .particles
            .iter_mut()
            .enumerate()
            .map(|(i, (pose, p))| (i, p.error(pose)))
            .collect();
        let weights = invert_errors(&errors);
        let (best_pose, best_map) = &self.particles[weights.mode().unwrap()];
        self.best_particle = ((*best_pose).into(), best_map.clone());
        let mut new_particles = vec![];
        for _ in 0..self.particles.len() {
            let choice = weights.pick_random_key();
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

#[derive(Copy, Clone, Default, Debug)]
pub struct PoseEstimate {
    last_raw: Option<RobotPose>,
    current_estimate: RobotPose,
}

impl From<PoseEstimate> for RobotPose {
    fn from(value: PoseEstimate) -> Self {
        value.current_estimate
    }
}

impl PoseEstimate {
    pub fn updated_raw_pose(&mut self, raw_pose: RobotPose) {
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
        self.current_estimate = map.noise(self.current_estimate, sensor_info)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Degrees, FloatPoint, PoseEstimate, RobotPose};

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
            let estimated: RobotPose = estimate.into();
            assert_eq!(pose, estimated);
        }
    }
}
