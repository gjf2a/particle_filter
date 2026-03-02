// New type of particle filter - the consistent particle filter
// It will reject any inconsistent maps but keep all the others.

use std::{cmp::Ordering, collections::BTreeSet, iter::repeat_n};

use hash_histogram::HashHistogram;

use crate::{PoseEstimate, Radians, RobotPose, SensorNoiseMap};

pub trait ConsistentMap: SensorNoiseMap {
    type StatType: StatCollector<Self>;

    fn is_consistent(&self) -> bool;
}

pub trait StatCollector<M>: Default + Clone {
    fn gather_data_from(&mut self, iteration: usize, particle: &M);
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

#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum SelectionStrategy {
    #[default]
    Uniform,
    DistanceWeight,
}

pub struct ConsistentParticleFilter<M: ConsistentMap> {
    particles: Vec<ConsistentParticle<M>>,
    total_iterations: usize,
    stats: M::StatType,
    example_failure: Option<ConsistentParticle<M>>,
    selection_strategy: SelectionStrategy,
}

impl<M: ConsistentMap> ConsistentParticleFilter<M> {
    pub fn new(
        num_particles: usize,
        starting_map: &M,
        selection_strategy: SelectionStrategy,
    ) -> Self {
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
            stats: M::StatType::default(),
            example_failure: None,
            selection_strategy,
        }
    }

    pub fn total_iterations(&self) -> usize {
        self.total_iterations
    }

    pub fn stats(&self) -> M::StatType {
        self.stats.clone()
    }

    pub fn example_failure(&self) -> Option<ConsistentParticle<M>> {
        self.example_failure.clone()
    }

    pub fn failed(&self) -> bool {
        self.example_failure.is_some()
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
        self.update_all_particles(new_raw_pose, sensor_info);
        let consistent = self.find_consistent_particles();
        if 0 < consistent.len() && consistent.len() < self.particles.len() {
            self.repopulate(&consistent, sensor_info);
        } else {
            self.example_failure = Some(self.particles[0].clone());
        }
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

    fn find_consistent_particles(&mut self) -> BTreeSet<usize> {
        let mut consistent = BTreeSet::new();
        for i in 0..self.particles.len() {
            if self.particles[i].map.is_consistent() {
                consistent.insert(i);
            } else {
                self.stats
                    .gather_data_from(self.total_iterations, &self.particles[i].map);
            }
        }
        consistent
    }

    fn repopulate(
        &mut self,
        consistent: &BTreeSet<usize>,
        sensor_info: Option<&M::SensorType>,
    ) {
        let weights = self.get_consistent_weights(consistent);
        let mut new_particles = consistent
            .iter()
            .map(|i| self.particles[*i].clone())
            .collect::<Vec<_>>();
        while new_particles.len() < self.particles.len() {
            let choice = weights.pick_random_key();
            let mut new_particle = self.particles[choice].clone();
            new_particle.add_noise(sensor_info);
            new_particles.push(new_particle);
        }
        std::mem::swap(&mut new_particles, &mut self.particles);
    }

    fn get_consistent_weights(&self, consistent: &BTreeSet<usize>) -> HashHistogram<usize, f64> {
        let inconsistent = (0..self.particles.len())
            .filter(|i| !consistent.contains(&i))
            .collect::<Vec<_>>();
        let mut weights = HashHistogram::new();
        for c in consistent.iter() {
            let weight = match self.selection_strategy {
                SelectionStrategy::Uniform => 1.0,
                _ => inconsistent
                    .iter()
                    .map(|i| {
                        self.particles[*i]
                            .estimated_pose()
                            .pos
                            .euclidean_distance(self.particles[*c].estimated_pose().pos)
                    })
                    .min_by(|d1, d2| d1.partial_cmp(d2).unwrap_or(Ordering::Equal))
                    .unwrap(),
            };
            weights.bump_by(c, weight);
        }
        weights
    }
}
