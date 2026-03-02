use std::iter::repeat_n;

use rand::{rng, seq::IndexedRandom};

use crate::{PoseEstimate, Radians, RobotPose, SensorNoiseMap};

#[derive(Clone)]
pub struct InconsistentParticle<M: SensorNoiseMap> {
    estimate: PoseEstimate,
    map: M,
    parent: Option<usize>,
}

impl<M: SensorNoiseMap> InconsistentParticle<M> {
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

pub struct InconsistentParticleFilter<M: SensorNoiseMap> {
    particles: Vec<InconsistentParticle<M>>,
    total_iterations: usize,
    example_failure: Option<InconsistentParticle<M>>,
}

impl<M: SensorNoiseMap> InconsistentParticleFilter<M> {
    pub fn new(
        num_particles: usize,
        starting_map: &M,
    ) -> Self {
        let mut aliases = Vec::with_capacity(num_particles * (num_particles + 1) / 2);
        for i in 0..num_particles {
            for _ in 0..=i {
                aliases.push(i);
            }
        }
        let particles = repeat_n(InconsistentParticle::new(starting_map), num_particles).collect();
        Self {
            particles,
            total_iterations: 0,
            example_failure: None,
        }
    }

    pub fn total_iterations(&self) -> usize {
        self.total_iterations
    }

    pub fn example_failure(&self) -> Option<InconsistentParticle<M>> {
        self.example_failure.clone()
    }

    pub fn failed(&self) -> bool {
        self.particles.len() == 0
    }

    pub fn particles(&self) -> impl Iterator<Item = &InconsistentParticle<M>> {
        self.particles.iter()
    }

    pub fn iterate(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        sensor_info: Option<&M::SensorType>,
    ) {
        self.example_failure = None;
        self.total_iterations += 1;
        self.update_all_particles(new_raw_pose, sensor_info);
        self.repopulate(sensor_info);
    }

    fn update_all_particles(
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

    fn repopulate(
        &mut self,
        sensor_info: Option<&M::SensorType>,
    ) {
        let mut new_particles = vec![];
        let mut rng = rng();
        while new_particles.len() < self.particles.len() {
            let choice = self.particles.choose(&mut rng).unwrap();
            let mut new_particle = choice.clone();
            new_particle.add_noise(sensor_info);
            new_particles.push(new_particle);
        }
        std::mem::swap(&mut new_particles, &mut self.particles);
    }
}
