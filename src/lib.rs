pub mod nums;
pub mod point;

pub use nums::*;
pub use point::*;

use hash_histogram::HashHistogram;
use rand_distr::{Distribution, Normal};

pub trait ObstacleMap: Clone + Default {
    type SensorType;

    fn error(&mut self, estimated_pose: &RobotPose) -> f64;
    fn sensor_update(&mut self, estimated_pose: &RobotPose, sensor_info: Option<&Self::SensorType>);

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
    particles: Vec<(RobotPose, M)>,
    best_particle: (RobotPose, M),
    last_raw_pose: Option<RobotPose>,
}

impl<M: ObstacleMap> ParticleFilter<M> {
    pub fn new(num_particles: usize) -> Self {
        Self {
            particles: std::iter::repeat((RobotPose::default(), M::default()))
                .take(num_particles)
                .collect(),
            best_particle: (RobotPose::default(), M::default()),
            last_raw_pose: None,
        }
    }

    pub fn current_best(&self) -> (RobotPose, M) {
        self.best_particle.clone()
    }

    pub fn iterate(&mut self, new_raw_pose: Option<RobotPose>, sensor_info: Option<&M::SensorType>) {
        self.resample();
        self.update_all(new_raw_pose, sensor_info);
    }

    fn update_all(&mut self, new_raw_pose: Option<RobotPose>, sensor_info: Option<&M::SensorType>) {
        for (pose, particle) in self.particles.iter_mut() {
            particle.sensor_update(pose, sensor_info);
            let current_estimate =
                Self::current_estimated_pose_for(&mut self.last_raw_pose, new_raw_pose, pose);
            *pose = particle.noise(current_estimate, sensor_info);
        }
    }

    fn current_estimated_pose_for(
        last_raw_pose: &mut Option<RobotPose>,
        new_raw_pose: Option<RobotPose>,
        last_estimated_pose: &RobotPose,
    ) -> RobotPose {
        let mut current_estimated_pose = *last_estimated_pose;
        if let Some(raw_pose) = new_raw_pose {
            if let Some(prev_pose) = last_raw_pose {
                current_estimated_pose += raw_pose - *prev_pose;
            }
            *last_raw_pose = Some(raw_pose);
        }
        current_estimated_pose
    }

    fn resample(&mut self) {
        let errors: HashHistogram<usize, f64> = self
            .particles
            .iter_mut()
            .enumerate()
            .map(|(i, (pose, p))| (i, p.error(pose)))
            .collect();
        let weights = invert_errors(&errors);
        self.best_particle = self.particles[weights.mode().unwrap()].clone();
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
