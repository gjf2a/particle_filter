// New type of particle filter - the consistent particle filter
// It will reject any inconsistent maps but keep all the others.

use std::{collections::{BTreeMap, BTreeSet}, iter::repeat_n};

use hash_histogram::HashHistogram;
use rand::{RngExt, rng};

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
    DistanceRank
}

pub struct ConsistentParticleFilter<M: ConsistentMap> {
    particles: Vec<ConsistentParticle<M>>,
    total_iterations: usize,
    stats: M::StatType,
    example_failure: Option<ConsistentParticle<M>>,
    selection_strategy: SelectionStrategy,
}

impl<M: ConsistentMap> ConsistentParticleFilter<M> {
    pub fn new(num_particles: usize, starting_map: &M, selection_strategy: SelectionStrategy) -> Self {
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
            selection_strategy
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
        self.example_failure = None;
        self.total_iterations += 1;
        self.update_all_particles(new_raw_pose, sensor_info);
        let consistent = self.find_consistent_particles();
        let num_particles = self.particles.len();
        if 0 < consistent.len() && consistent.len() < num_particles {
            self.repopulate(num_particles, &consistent, sensor_info);
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
                if self.example_failure.is_none() {
                    self.example_failure = Some(self.particles[i].clone());
                }
            }
        }
        consistent
    }

    fn repopulate(
        &mut self,
        num_particles: usize,
        consistent: &BTreeSet<usize>,
        sensor_info: Option<&M::SensorType>,
    ) {
        let inconsistent = (0..num_particles).filter(|i| !consistent.contains(&i)).collect::<Vec<_>>();        
        let mut weights = HashHistogram::new();
        for c in consistent.iter() {
            let weight = match self.selection_strategy {
                SelectionStrategy::Uniform => 1.0,
                _ => inconsistent.iter().map(|i| self.particles[*i].estimated_pose().pos.euclidean_distance(self.particles[*c].estimated_pose().pos)).min_by(|d1, d2| d1.partial_cmp(d2).unwrap_or(std::cmp::Ordering::Equal)).unwrap()                
            };
            weights.bump_by(c, weight);
        }
        self.particles = consistent
            .iter()
            .map(|i| self.particles[*i].clone())
            .collect();
        while self.particles.len() < num_particles {
            let choice = weights.pick_random_key();
            let mut new_particle = self.particles[choice].clone();
            new_particle.add_noise(sensor_info);
            self.particles.push(new_particle);
        }
    }
}
