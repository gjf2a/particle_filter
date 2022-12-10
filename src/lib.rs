use std::cmp::max;
use std::fmt::Debug;
use ordered_float::OrderedFloat;
use distribution_select::Distribution;
pub use crate::position_types::{PolarCoord, RobotPosition};

pub mod position_types;
pub mod sonar3bot;
mod grid_map;
mod sonar;

pub trait SensorCorrection : Clone + Debug + Eq + Ord + PartialEq + PartialOrd {
    type SensorReading: Clone;

    fn fit(&self, position: &RobotPosition, reading: &Self::SensorReading) -> f64;

    fn update_from(&mut self, position: &RobotPosition, reading: &Self::SensorReading);
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Particle<C: SensorCorrection<SensorReading=S>, S: Clone + Debug + Eq + PartialEq + Ord + PartialOrd> {
    pos: RobotPosition,
    map: C
}

#[derive(Clone)]
pub struct ParticleFilter<C: SensorCorrection<SensorReading=S>, S: Clone + Debug + Eq + PartialEq + Ord + PartialOrd, N: Fn(PolarCoord) -> PolarCoord> {
    particles: Vec<Particle<C, S>>,
    noise_function: N,
    best: Particle<C,S>
}

impl <C: SensorCorrection<SensorReading=S>, S: Clone + Debug + Eq + PartialEq + Ord + PartialOrd, N: Fn(PolarCoord) -> PolarCoord> ParticleFilter<C, S, N> {
    pub fn new<P: Fn() -> Particle<C, S>>(num_particles: usize, noise_function: N, particle_maker: P) -> Self {
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
        let particle_fits: Vec<(&Particle<C,S>, f64)> = self.particles.iter()
            .map(|p| (p, p.map.fit(&p.pos.updated_by(motion), measurement))).collect();
        let distro = Self::make_distro_from(&particle_fits);
        self.best = self.get_best_from(&particle_fits);
        let num_particles = self.particles.len();
        self.particles = (0..num_particles).map(|_| distro.random_pick()).collect();
    }

    fn make_distro_from(particle_fits: &Vec<(&Particle<C,S>, f64)>) -> Distribution<Particle<C,S>> {
        let mut distro: Distribution<Particle<C,S>> = Distribution::new();
        for (particle, fit) in particle_fits.iter() {
            let fit = 1.0 + max(OrderedFloat(0.0), OrderedFloat(*fit)).into_inner();
            distro.add(particle, fit);
        }
        distro
    }

    fn get_best_from(&self, particle_fits: &Vec<(&Particle<C,S>, f64)>) -> Particle<C, S> {
        let mut best = 0;
        for i in 1..particle_fits.len() {
            if particle_fits[i].1 > particle_fits[best].1 {
                best = i;
            }
        }
        particle_fits[best].0.clone()
    }
}
