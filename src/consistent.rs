// New type of particle filter - the consistent particle filter
// It will reject any inconsistent maps but keep all the others.

use std::iter::repeat_n;

use rand::{RngExt, rng};

use crate::{PoseEstimate, Radians, RobotPose, SensorNoiseMap};

use hash_histogram::HashHistogram;

pub trait ConsistentMap: SensorNoiseMap {
    fn is_consistent(&self) -> bool;
}

#[derive(Clone)]
pub struct ConsistentParticle<M: ConsistentMap> {
    estimate: PoseEstimate,
    map: M,
    parent: Option<usize>,
}

impl<M: ConsistentMap> ConsistentParticle<M> {
    pub fn estimated_pose(&self) -> RobotPose<Radians> {
        self.estimate.into()
    }

    pub fn map(&self) -> &M {
        &self.map
    }

    pub fn parent_index(&self) -> Option<usize> {
        self.parent
    }

    fn new(starting_map: &M) -> Self {
        Self {
            estimate: PoseEstimate::default(),
            map: starting_map.clone(),
            parent: None,
        }
    }

    fn add_noise(&mut self, sensor_info: Option<&M::SensorType>) {
        self.estimate.add_noise(&self.map, sensor_info);
    }

    fn sensor_update(&mut self, sensor_info: Option<&M::SensorType>) {
        self.add_noise(sensor_info);
        self.map.sensor_update(self.estimated_pose(), sensor_info);
    }
}

pub struct ConsistentParticleFilter<M: ConsistentMap> {
    particles: Vec<ConsistentParticle<M>>,
    total_iterations: usize,
    iteration_inconsistencies: HashHistogram<usize, usize>,
}

impl<M: ConsistentMap> ConsistentParticleFilter<M> {
    pub fn new(num_particles: usize, starting_map: &M) -> Self {
        let mut aliases = Vec::with_capacity(num_particles * (num_particles + 1) / 2);
        for i in 0..num_particles {
            for _ in 0..=i {
                aliases.push(i);
            }
        }
        let particles = repeat_n(ConsistentParticle::new(starting_map), num_particles).collect();
        Self {
            particles,
            total_iterations: 0,
            iteration_inconsistencies: HashHistogram::default(),
        }
    }

    pub fn iteration_inconsistencies(&self) -> HashHistogram<usize, usize> {
        self.iteration_inconsistencies.clone()
    }

    pub fn failed(&self) -> bool {
        self.particles.len() == 0
    }

    pub fn particles(&self) -> impl Iterator<Item = &ConsistentParticle<M>> {
        self.particles.iter()
    }

    pub fn iterate(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        sensor_info: Option<&M::SensorType>,
    ) {
        self.total_iterations += 1;
        for particle in self.particles.iter_mut() {
            if let Some(raw_pose) = new_raw_pose {
                particle.estimate.updated_raw_pose(raw_pose);
            }
            particle.sensor_update(sensor_info);
        }

        let consistent = (0..self.particles.len())
            .filter(|i| self.particles[*i].map.is_consistent())
            .collect::<Vec<_>>();
        let num_particles = self.particles.len();
        if consistent.len() < num_particles {
            self.iteration_inconsistencies
                .bump_by(&self.total_iterations, num_particles - consistent.len());
            self.particles = consistent
                .iter()
                .map(|i| self.particles[*i].clone())
                .collect();
            if !self.failed() {
                let mut rng = rng();
                while self.particles.len() < num_particles {
                    let choice = rng.random_range(0..self.particles.len());
                    let mut new_particle = self.particles[choice].clone();
                    new_particle.add_noise(sensor_info);
                    self.particles.push(new_particle);
                }
            }
        }
    }
}
