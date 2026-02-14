pub mod point;

use hash_histogram::HashHistogram;
use point::FloatPoint;

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose {
    pub pos: FloatPoint,
    pub theta: f64,
}

pub trait Particle : Clone {
    fn weight(&self) -> f64;
    fn pose(&self) -> RobotPose;
    fn update<S>(&mut self, new_pose: RobotPose, sensor_info: &S);
}

#[derive(Clone, Debug)]
pub struct ParticleFilter<P: Particle, NoiseFunc: Clone + Fn(RobotPose) -> RobotPose> {
    particles: Vec<P>,
    best_particle: P,
    noise_func: NoiseFunc,
}

impl<MapType: Particle, NoiseFunc: Clone + Fn(RobotPose) -> RobotPose> ParticleFilter<MapType, NoiseFunc> {
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

    pub fn iterate<S>(&mut self, sensor_info: &S) {
        self.resample();
        for particle in self.particles.iter_mut() {
            particle.update((self.noise_func)(particle.pose()), sensor_info);
        }
    }

    fn resample(&mut self) {
        let mut distro = HashHistogram::new();
        let mut best_weight = 0.0;
        let mut best_i = 0;
        for (i, particle) in self.particles.iter().enumerate() {
            let weight = particle.weight();
            if weight > best_weight {
                best_weight = weight;
                best_i = i;
            }
            distro.bump_by(&i, weight + 1.0);
        }

        self.best_particle = self.particles[best_i].clone();

        let mut new_particles = vec![];
        for _ in 0..self.particles.len() {
            let choice = distro.pick_random_key();
            new_particles.push(self.particles[choice].clone());
        }

        std::mem::swap(&mut new_particles, &mut self.particles);
    }
}