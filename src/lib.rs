pub mod point;
pub mod nums;

use hash_histogram::HashHistogram;

use crate::nums::RobotPose;

pub trait Particle: Clone {
    type SensorType;

    fn error(&self) -> f64;
    fn pose(&self) -> RobotPose;
    fn update(&mut self, sensor_info: &Self::SensorType);
    fn add_noise<N: Fn(RobotPose, &Self::SensorType) -> RobotPose>(&mut self, noise_func: N);
}

#[derive(Clone, Debug)]
pub struct ParticleFilter<P: Particle, NoiseFunc: Clone> {
    particles: Vec<P>,
    best_particle: P,
    noise_func: NoiseFunc,
}

impl<MapType: Particle, NoiseFunc: Clone> ParticleFilter<MapType, NoiseFunc> {
    pub fn new(initial_map: &MapType, num_particles: usize, noise_func: NoiseFunc) -> Self {
        Self {
            particles: std::iter::repeat(initial_map.clone())
                .take(num_particles)
                .collect(),
            best_particle: initial_map.clone(),
            noise_func,
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
            particle.update(sensor_info);
            particle.add_noise(self.noise_func.clone());
        }
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
