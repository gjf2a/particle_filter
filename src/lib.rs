use std::cmp::max;
use ordered_float::OrderedFloat;
use crate::distribution::Distribution;
pub use crate::position_types::{PolarCoord, RobotPosition};

mod position_types;
mod grid_map;
mod distribution;
mod sonar;

pub trait SensorMap {
    type SensorReading;

    fn fit(&self, position: &RobotPosition, reading: &Self::SensorReading) -> f64;

    fn update_from(&mut self, position: &RobotPosition, reading: &Self::SensorReading);
}

#[derive(Clone)]
pub struct Particle<M: Clone+SensorMap<SensorReading=S>, S:Clone> {
    pos: RobotPosition,
    map: M
}

#[derive(Clone)]
pub struct ParticleFilter<M: Clone+SensorMap<SensorReading=S>, S: Clone, N: Fn(PolarCoord) -> PolarCoord> {
    particles: Vec<Particle<M, S>>,
    noise_function: N,
    best: Particle<M,S>
}

impl <M: Clone + SensorMap<SensorReading=S>, S: Clone, N: Fn(PolarCoord) -> PolarCoord> ParticleFilter<M, S, N> {
    pub fn new<P: Fn() -> Particle<M, S>>(num_particles: usize, noise_function: N, particle_maker: P) -> Self {
        ParticleFilter {
            particles: (0..num_particles).map(|_| particle_maker()).collect(),
            noise_function,
            best: particle_maker()
        }
    }

    pub fn iterate(&mut self, measurement: &S, motion: PolarCoord) {
        self.resample(measurement, motion);
        self.add_measurement(measurement, motion);
    }

    fn add_measurement(&mut self, measurement: &S, motion: PolarCoord) {
        for particle in self.particles.iter_mut() {
            particle.pos.updated_by((self.noise_function)(motion));
            particle.map.update_from(&particle.pos, measurement);
        }
    }

    fn resample(&mut self, measurement: &S, motion: PolarCoord) {
        let particle_fits: Vec<(&Particle<M,S>, f64)> = self.particles.iter()
            .map(|p| (p, p.map.fit(&p.pos.updated_by(motion), measurement))).collect();
        let distro = Self::make_distro_from(&particle_fits);
        self.best = self.get_best_from(&particle_fits);
        let num_particles = self.particles.len();
        self.particles = (0..num_particles).map(|_| distro.random_pick()).collect();
    }

    fn make_distro_from(particle_fits: &Vec<(&Particle<M,S>, f64)>) -> Distribution<Particle<M,S>> {
        let mut distro: Distribution<Particle<M,S>> = Distribution::new();
        for (particle, fit) in particle_fits.iter() {
            let fit = 1.0 + max(OrderedFloat(0.0), OrderedFloat(*fit)).into_inner();
            distro.add(particle, fit);
        }
        distro
    }

    fn get_best_from(&self, particle_fits: &Vec<(&Particle<M,S>, f64)>) -> Particle<M, S> {
        let mut best = 0;
        for i in 1..particle_fits.len() {
            if particle_fits[i].1 > particle_fits[best].1 {
                best = i;
            }
        }
        particle_fits[best].0.clone()
    }
}
