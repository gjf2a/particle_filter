pub mod bit_grid_map;
pub mod stats;
pub mod walker;

pub use bit_grid_map::*;
use bits::BitArray;
use enum_iterator::Sequence;
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
pub struct Particle {
    estimate: PoseEstimate,
    map: BitGridMap,
    parent: Option<usize>,
    noises: Noises,
}

impl Particle {
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

#[derive(Copy, Clone, PartialEq, Eq, Sequence, Debug)]
pub enum SelectionStrategy {
    Weighted,
    RankProportion,
}

impl SelectionStrategy {
    pub fn selector(&self, weights: &HashHistogram<usize, f64>) -> WalkerAlias {
        match self {
            Self::RankProportion => WalkerAlias::rank_proportionate(weights.len()),
            Self::Weighted => WalkerAlias::weighted(weights),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Sequence, Debug)]
pub enum WeightStrategy {
    Uniform,
    Compactness,
    MinPose,
    MinSpaceDifference,
    MinObstacleDifference,
}

impl WeightStrategy {
    pub fn weights(
        &self,
        particles: &Vec<Particle>,
        inconsistent: &Vec<Particle>,
    ) -> HashHistogram<usize, f64> {
        let mut weights = HashHistogram::new();
        for (i, p) in particles.iter().enumerate() {
            let weight = match self {
                Self::Uniform => 1.0,
                Self::MinPose => Self::min_distance_to_any_of(p, &inconsistent),
                _ => todo!(),
            };
            weights.bump_by(&i, weight);
        }
        weights
    }

    fn min_distance_to_any_of(p: &Particle, inconsistent: &Vec<Particle>) -> f64 {
        inconsistent
            .iter()
            .map(|i| {
                i.estimated_pose()
                    .pos
                    .euclidean_distance(p.estimated_pose().pos)
            })
            .min_by(|d1, d2| d1.partial_cmp(d2).unwrap_or(Ordering::Equal))
            .unwrap()
    }
}

#[derive(Copy, Clone, PartialEq)]
pub struct ParticleFilterSettings {
    pub noises: Noises,
    pub num_particles: usize,
    pub square_size_m: f64,
    pub robot_radius_m: f64,
    pub selection_strategy: SelectionStrategy,
    pub weight_strategy: WeightStrategy,
}

#[derive(Clone)]
pub struct ParticleFilter {
    last_raw: Option<RobotPose<Radians>>,
    particles: Vec<Particle>,
    total_iterations: usize,
    stats: BitGridStats,
    example_failure: Option<Particle>,
    selection_strategy: SelectionStrategy,
    weight_strategy: WeightStrategy,
}

impl ParticleFilter {
    pub fn new(settings: ParticleFilterSettings) -> Self {
        let particles = repeat_n(
            Particle::new(
                settings.square_size_m,
                settings.robot_radius_m,
                settings.noises,
            ),
            settings.num_particles,
        )
        .collect();
        Self {
            last_raw: None,
            particles,
            total_iterations: 0,
            stats: BitGridStats::default(),
            example_failure: None,
            selection_strategy: settings.selection_strategy,
            weight_strategy: settings.weight_strategy,
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

    pub fn example_failure(&self) -> Option<Particle> {
        self.example_failure.clone()
    }

    pub fn failed(&self) -> bool {
        self.example_failure.is_some()
    }

    pub fn particles(&self) -> impl Iterator<Item = &Particle> {
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
        if consistent.count_ones() == 0 {
            self.example_failure = Some(self.particles[0].clone());
        } else if consistent.count_ones() < self.particles.len() {
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

    fn find_consistent_particles(&mut self) -> BitArray {
        (0..self.particles.len())
            .filter(|i| self.particles[*i].map.is_consistent())
            .collect()
    }

    fn repopulate(&mut self, consistent: BitArray, obstacle: Option<FloatPoint>) {
        let num_particles = self.particles.len();
        let ones = BitArray::ones(num_particles);
        let inconsistent = (&consistent ^ &ones)
            .one_indices()
            .inspect(|i| {
                self.stats
                    .gather_data_from(self.total_iterations, &self.particles[*i].map)
            })
            .map(|i| self.particles[i].clone())
            .collect::<Vec<_>>();

        self.particles = consistent
            .one_indices()
            .map(|i| self.particles[i].clone())
            .collect::<Vec<_>>();
        let selector = self.make_selector(&inconsistent);

        while self.particles.len() < num_particles {
            let choice = selector.choose();
            let mut new_particle = self.particles[choice].clone();
            new_particle.add_noise(obstacle);
            self.particles.push(new_particle);
        }
    }

    fn make_selector(&mut self, inconsistent: &Vec<Particle>) -> WalkerAlias {
        let weights = self
            .weight_strategy
            .weights(&self.particles, &inconsistent)
            .ranking_with_counts();
        self.sort_particles_by(weights.iter().map(|(i, _)| *i));
        let weights = weights
            .iter()
            .map(|(_, w)| *w)
            .enumerate()
            .collect::<HashHistogram<usize, f64>>();
        self.selection_strategy.selector(&weights)
    }

    fn sort_particles_by<I: Iterator<Item = usize>>(&mut self, permutation: I) {
        self.particles = permutation
            .map(|current| self.particles[current].clone())
            .collect();
    }
}

impl Index<usize> for ParticleFilter {
    type Output = Particle;

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
