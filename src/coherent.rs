// New type of particle filter - the coherent filter
// It will reject any "incoherent" maps but keep all the others.

use std::iter::repeat_n;

use rand::{RngExt, rng};

use crate::{PoseEstimate, Radians, RobotPose, SensorNoiseMap};

pub trait CoherenceMap: SensorNoiseMap {
    fn is_coherent(&self) -> bool;
}

#[derive(Clone)]
pub struct CParticle<M: CoherenceMap> {
    estimate: PoseEstimate,
    map: M,
    parent: Option<usize>,
}

impl<M: CoherenceMap> CParticle<M> {
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

    fn sensor_update(&mut self, sensor_info: Option<&M::SensorType>) {
        self.estimate.add_noise(&self.map, sensor_info);
        self.map.sensor_update(self.estimated_pose(), sensor_info);
    }
}

pub struct CParticleFilter<M: CoherenceMap> {
    particles: Vec<CParticle<M>>,
}

impl<M: CoherenceMap> CParticleFilter<M> {
    pub fn new(num_particles: usize, starting_map: &M) -> Self {
        let mut aliases = Vec::with_capacity(num_particles * (num_particles + 1) / 2);
        for i in 0..num_particles {
            for _ in 0..=i {
                aliases.push(i);
            }
        }
        let particles = repeat_n(CParticle::new(starting_map), num_particles).collect();
        Self { particles }
    }

    pub fn failed(&self) -> bool {
        self.particles.len() == 0
    }

    pub fn particles(&self) -> impl Iterator<Item = &CParticle<M>> {
        self.particles.iter()
    }

    pub fn iterate(
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

        let coherent = (0..self.particles.len())
            .filter(|i| self.particles[*i].map.is_coherent())
            .collect::<Vec<_>>();
        let num_particles = self.particles.len();
        if coherent.len() < num_particles {
            self.particles = coherent
                .iter()
                .map(|i| self.particles[*i].clone())
                .collect();
            if !self.failed() {
                let mut rng = rng();
                while self.particles.len() < num_particles {
                    let choice = rng.random_range(0..self.particles.len());
                    self.particles.push(self.particles[choice].clone());
                }
            }
        }
    }
}
