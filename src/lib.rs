pub mod nums;
pub mod point;

pub use nums::*;
pub use point::*;

use hash_histogram::HashHistogram;

pub trait Sensor {
    fn current_pose(&self) -> Option<RobotPose>;
}

pub trait Particle: Clone {
    type SensorType: Sensor;

    fn error(&self) -> f64;
    fn pose(&self) -> RobotPose;
    fn set_pose(&mut self, new_pose: RobotPose);
    fn sensor_update(&mut self, sensor_info: &Self::SensorType);
}

#[derive(Clone, Debug)]
pub struct ParticleFilter<P: Particle, NoiseFunc: Clone> {
    particles: Vec<P>,
    best_particle: P,
    noise_func: NoiseFunc,
    last_raw_pose: Option<RobotPose>,
}

impl<MapType: Particle, NoiseFunc: Clone> ParticleFilter<MapType, NoiseFunc> {
    pub fn new(initial_map: &MapType, num_particles: usize, noise_func: NoiseFunc) -> Self {
        Self {
            particles: std::iter::repeat(initial_map.clone())
                .take(num_particles)
                .collect(),
            best_particle: initial_map.clone(),
            noise_func,
            last_raw_pose: None,
        }
    }

    pub fn current_best(&self) -> MapType {
        self.best_particle.clone()
    }

    pub fn iterate(&mut self, sensor_info: &MapType::SensorType)
    where
        NoiseFunc: Fn(RobotPose, &MapType::SensorType) -> RobotPose,
    {
        self.resample();
        for particle in self.particles.iter_mut() {
            particle.sensor_update(sensor_info);
            let current_estimate = Self::current_estimated_pose_for(&mut self.last_raw_pose, particle, sensor_info);
            particle.set_pose((self.noise_func)(current_estimate, sensor_info));
        }
    }

    fn current_estimated_pose_for(last_raw_pose: &mut Option<RobotPose>, particle: &MapType, sensor_info: &MapType::SensorType) -> RobotPose {
        let mut current_estimated_pose = particle.pose();
        if let Some(raw_pose) = sensor_info.current_pose() {
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
            .iter()
            .enumerate()
            .map(|(i, p)| (i, p.error()))
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
