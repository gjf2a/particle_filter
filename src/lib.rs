pub mod bit_grid_map;
pub mod stats;
pub mod walker;

pub use bit_grid_map::*;
use rand::{RngExt, rng};
pub use stats::*;

use bit_grid::{
    angle::{Degrees, Radians},
    point::FloatPoint,
    pose::RobotPose,
};
use hash_histogram::HashHistogram;
use rand_distr::{Distribution, Normal};
use std::fmt::Debug;
use std::{cmp::Ordering, iter::repeat_n, ops::Index};

use crate::walker::WalkerAlias;

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct Noises {
    pub odom: Noise,
    pub obst: Noise,
}

impl Noises {
    fn noise(&self, obstacle: Option<FloatPoint>) -> Noise {
        match obstacle {
            None => self.odom,
            Some(_) => self.obst,
        }
    }
}

#[derive(Clone)]
pub struct ConsistentParticle {
    estimate: PoseEstimate,
    map: BitGridMap,
    parent: Option<usize>,
    noises: Noises,
}

impl ConsistentParticle {
    pub fn estimated_pose(&self) -> RobotPose<Radians> {
        self.estimate.into()
    }

    pub fn map(&self) -> &BitGridMap {
        &self.map
    }

    pub fn parent_index(&self) -> Option<usize> {
        self.parent
    }

    fn new(square_size_m: f64, robot_radius_m: f64, noises: Noises) -> Self {
        Self {
            estimate: PoseEstimate::default(),
            map: BitGridMap::new(square_size_m, robot_radius_m),
            parent: None,
            noises,
        }
    }

    fn add_noise(&mut self, obstacle: Option<FloatPoint>) {
        let noise = self.noises.noise(obstacle);
        self.estimate.add_noise(noise);
    }

    fn add_sensed_obstacles(&mut self, obstacle: Option<FloatPoint>) {
        if let Some(obstacle) = obstacle {
            self.map
                .add_obstacle_at(&self.estimate.update_other_point(&obstacle));
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum SelectionStrategy {
    Weighted,
    RankProportion,
}

impl SelectionStrategy {
    pub fn selector(&self, weights: &HashHistogram<usize, f64>) -> WalkerAlias {
        match self {
            Self::RankProportion => WalkerAlias::rank_proportionate(weights.len()),
            Self::Weighted => WalkerAlias::weighted(weights)
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum WeightStrategy {
    Uniform,
    Compactness,
    MinPose,
    MinSpaceDifference,
    MinObstacleDifference,
}

#[derive(Clone)]
pub struct ConsistentParticleFilter {
    last_raw: Option<RobotPose<Radians>>,
    particles: Vec<ConsistentParticle>,
    total_iterations: usize,
    stats: BitGridStats,
    example_failure: Option<ConsistentParticle>,
    selection_strategy: SelectionStrategy,
    weight_strategy: WeightStrategy,
}

impl ConsistentParticleFilter {
    pub fn new(
        num_particles: usize,
        square_size_m: f64,
        robot_radius_m: f64,
        noises: Noises,
        selection_strategy: SelectionStrategy,
        weight_strategy: WeightStrategy,
    ) -> Self {
        let particles = repeat_n(
            ConsistentParticle::new(square_size_m, robot_radius_m, noises),
            num_particles,
        )
        .collect();
        Self {
            last_raw: None,
            particles,
            total_iterations: 0,
            stats: BitGridStats::default(),
            example_failure: None,
            selection_strategy,
            weight_strategy,
        }
    }

    pub fn len(&self) -> usize {
        self.particles.len()
    }

    pub fn last_raw_pose(&self) -> Option<RobotPose<Radians>> {
        self.last_raw
    }

    pub fn total_iterations(&self) -> usize {
        self.total_iterations
    }

    pub fn stats(&self) -> BitGridStats {
        self.stats.clone()
    }

    pub fn example_failure(&self) -> Option<ConsistentParticle> {
        self.example_failure.clone()
    }

    pub fn failed(&self) -> bool {
        self.example_failure.is_some()
    }

    pub fn particles(&self) -> impl Iterator<Item = &ConsistentParticle> {
        self.particles.iter()
    }

    pub fn iterate(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        obstacle: Option<FloatPoint>,
    ) {
        if new_raw_pose.is_some() {
            self.last_raw = new_raw_pose;
        }
        self.total_iterations += 1;
        self.update_all_particles(new_raw_pose, obstacle);
        let consistent = self.find_consistent_particles();
        if consistent.len() == 0 {
            self.example_failure = Some(self.particles[0].clone());
        } else if consistent.len() < self.particles.len() {
            self.repopulate(consistent, obstacle);
        }
    }

    fn update_all_particles(
        &mut self,
        new_raw_pose: Option<RobotPose<Radians>>,
        obstacle: Option<FloatPoint>,
    ) {
        for particle in self.particles.iter_mut() {
            if let Some(raw_pose) = new_raw_pose {
                particle.estimate.updated_raw_pose(raw_pose);
                particle
                    .map
                    .add_odometry_reading(&particle.estimated_pose().pos);
            }
            particle.add_noise(obstacle);
            particle.add_sensed_obstacles(obstacle);
        }
    }

    fn find_consistent_particles(&mut self) -> Vec<usize> {
        let mut consistent = Vec::new();
        for i in 0..self.particles.len() {
            if self.particles[i].map.is_consistent() {
                consistent.push(i);
            } else {
                self.stats
                    .gather_data_from(self.total_iterations, &self.particles[i].map);
            }
        }
        consistent
    }

    fn repopulate(&mut self, consistent: Vec<usize>, obstacle: Option<FloatPoint>) {
        let mut consistent = consistent;
        let weights = self.get_consistent_weights(&consistent);
        consistent.sort_by(|i, j| {
            weights
                .count(j)
                .partial_cmp(&weights.count(i))
                .unwrap_or(Ordering::Equal)
        });
        let selector = self.selection_strategy.selector(&weights);        
        let mut new_particles = consistent
            .iter()
            .map(|i| self.particles[*i].clone())
            .collect::<Vec<_>>();
        while new_particles.len() < self.particles.len() {
            let choice = selector.choose();
            let mut new_particle = self.particles[choice].clone();
            new_particle.add_noise(obstacle);
            new_particles.push(new_particle);
        }
        std::mem::swap(&mut new_particles, &mut self.particles);
    }

    fn get_consistent_weights(&self, consistent: &Vec<usize>) -> HashHistogram<usize, f64> {
        let inconsistent = (0..self.particles.len())
            .filter(|i| !consistent.contains(&i))
            .collect::<Vec<_>>();
        let mut weights = HashHistogram::new();
        for c in consistent.iter() {
            let weight = match self.weight_strategy {
                WeightStrategy::MinPose => {
                    self.min_distance_to_any_of(&self.particles[*c], &inconsistent)
                }
                _ => todo!(),
            };
            weights.bump_by(c, weight);
        }
        weights
    }

    fn min_distance_to_any_of(&self, p: &ConsistentParticle, inconsistent: &Vec<usize>) -> f64 {
        inconsistent
            .iter()
            .map(|i| {
                self.particles[*i]
                    .estimated_pose()
                    .pos
                    .euclidean_distance(p.estimated_pose().pos)
            })
            .min_by(|d1, d2| d1.partial_cmp(d2).unwrap_or(Ordering::Equal))
            .unwrap()
    }
}

impl Index<usize> for ConsistentParticleFilter {
    type Output = ConsistentParticle;

    fn index(&self, index: usize) -> &Self::Output {
        &self.particles[index]
    }
}

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct Noise {
    pub stdev_x_y: f64,
    pub stdev_angle: Degrees,
}

impl Noise {
    fn noise(&self, pose: RobotPose<Radians>) -> RobotPose<Radians> {
        let mut rng = rand::rng();
        let x_y_gaussian = Normal::new(0.0, self.stdev_x_y).unwrap();
        let theta_gaussian = Normal::new(0.0, self.stdev_angle.into()).unwrap();
        let x_y_noise =
            FloatPoint::new([x_y_gaussian.sample(&mut rng), x_y_gaussian.sample(&mut rng)]);
        let theta_noise = Degrees::new(theta_gaussian.sample(&mut rng));
        RobotPose {
            pos: (pose.pos + x_y_noise),
            theta: pose.theta + theta_noise.into(),
        }
    }
}

pub trait StatCollector<M>: Default + Clone {
    fn gather_data_from(&mut self, iteration: usize, particle: &M);
}

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub struct PoseEstimate {
    last_raw: Option<RobotPose<Radians>>,
    current_estimate: RobotPose<Radians>,
}

impl From<PoseEstimate> for RobotPose<Radians> {
    fn from(value: PoseEstimate) -> Self {
        value.current_estimate
    }
}

impl PoseEstimate {
    pub fn updated_raw_pose(&mut self, raw_pose: RobotPose<Radians>) {
        match self.last_raw {
            None => {
                self.current_estimate = raw_pose;
            }
            Some(last_raw) => {
                self.current_estimate += raw_pose - last_raw;
            }
        }
        self.last_raw = Some(raw_pose);
    }

    pub fn update_other_point(&self, pt: &FloatPoint) -> FloatPoint {
        let raw2estimated = self.current_estimate - self.last_raw.unwrap_or(RobotPose::default());
        raw2estimated.pos + *pt
    }

    pub fn add_noise(&mut self, noise: Noise) {
        self.current_estimate = noise.noise(self.current_estimate);
    }
}

pub struct Selector {
    selection_strategy: SelectionStrategy,
    weight_strategy: WeightStrategy,
    weights: HashHistogram<usize, f64>,
    walker_alias: WalkerAlias,
}
/*
impl Selector {
    pub fn setup(particle_filter: &ConsistentParticleFilter, consistent: &Vec<usize>) -> Self {

    }
}
    */

#[cfg(test)]
mod tests {
    use crate::{Degrees, FloatPoint, PoseEstimate, Radians, RobotPose};
    use bit_grid::point::Point;
    use bit_grid::pt;

    #[test]
    fn test_current_estimated_pose() {
        let mut estimate = PoseEstimate::default();
        for (x, y, theta) in [
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (1.0, 1.0, 90.0),
            (1.0, 2.0, 90.0),
        ] {
            let pose = RobotPose {
                pos: FloatPoint::new([x, y]),
                theta: Degrees::new(theta).into(),
            };
            estimate.updated_raw_pose(pose);
            let estimated: RobotPose<Radians> = estimate.into();
            assert_eq!(pose, estimated);
        }
    }

    #[test]
    fn test_other_point() {
        let test_pose_estimate = PoseEstimate {
            last_raw: Some(RobotPose {
                pos: pt!(1.0, 1.0),
                theta: Radians::new(0.0),
            }),
            current_estimate: RobotPose {
                pos: pt!(3.0, 2.0),
                theta: Radians::new(0.0),
            },
        };
        let expected = pt!(6.0, 4.0);
        assert_eq!(
            expected,
            test_pose_estimate.update_other_point(&pt!(4.0, 3.0))
        );
    }
}
