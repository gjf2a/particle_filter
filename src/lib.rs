pub mod angle;
pub mod bit_grid;
pub mod bit_grid_map;
pub mod bits;
pub mod irobot_create3;
pub mod path_plan;
pub mod point;
pub mod pose;
pub mod stats;
pub mod walker;

pub use bit_grid_map::*;
use bits::BitArray;
use enum_iterator::Sequence;
use serde::{Deserialize, Serialize};
pub use stats::*;

use angle::{Degrees, Radians};
use point::FloatPoint;
use pose::RobotPose;

use hash_histogram::HashHistogram;
use rand_distr::{Distribution, Normal};
use std::fmt::Debug;
use std::{cmp::Ordering, iter::repeat_n, ops::Index};

use walker::WalkerAliasTable;

use crate::bit_grid::BitGrid;

#[macro_export]
macro_rules! pt {
    ($x:expr, $y:expr) => {
        $crate::point::Point::new([$x, $y])
    };
}

#[derive(Copy, Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
pub struct Noises {
    pub clear: Noise,
    pub obst: Noise,
}

impl Noises {
    fn noise(&self, collision: bool) -> Noise {
        if collision {
            self.obst
        } else {
            self.clear
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Particle {
    estimate: PoseEstimate,
    map: BitGridMap,
    parent: Option<usize>,
    noises: Noises,
}

impl Particle {
    pub fn estimate(&self) -> &PoseEstimate {
        &self.estimate
    }

    pub fn estimated_pose(&self) -> RobotPose<Radians> {
        self.estimate.into()
    }

    pub fn map(&self) -> &BitGridMap {
        &self.map
    }

    pub fn robot_shadow(&self) -> BitGrid {
        self.map.robot_shadow(self.estimated_pose())
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

    fn add_noise(&mut self, collision: bool) {
        let noise = self.noises.noise(collision);
        self.estimate.add_noise(noise);
    }

    fn add_sensed_obstacles(&mut self, obstacle: Option<FloatPoint>) {
        if let Some(obstacle) = obstacle {
            self.map
                .add_obstacle_at(&self.estimate.update_other_point(&obstacle));
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Sequence, Debug, Serialize, Deserialize)]
pub enum SelectionStrategy {
    Weighted,
    RankProportion,
}

impl SelectionStrategy {
    pub fn selector(&self, weights: &HashHistogram<usize, f64>) -> WalkerAliasTable {
        match self {
            Self::RankProportion => WalkerAliasTable::rank_proportionate(weights.len()),
            Self::Weighted => WalkerAliasTable::weighted(weights),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Sequence, Debug, Serialize, Deserialize)]
pub enum WeightStrategy {
    Uniform,
    MinPose,
    BoundingBoxArea,
}

impl WeightStrategy {
    pub fn weights(
        &self,
        particles: &Vec<Particle>,
        inconsistent: &Vec<Particle>,
    ) -> HashHistogram<usize, f64> {
        let mut weights = HashHistogram::new();
        for (i, p) in particles.iter().enumerate() {
            let weight = self.weight(p, inconsistent);
            weights.bump_by(&i, weight);
        }
        if self.reverse_weights() {
            weights = reversed_weights(weights);
        }
        weights
    }

    fn reverse_weights(&self) -> bool {
        match self {
            Self::BoundingBoxArea => true,
            _ => false,
        }
    }

    fn weight(&self, p: &Particle, inconsistent: &Vec<Particle>) -> f64 {
        match self {
            Self::Uniform => 1.0,
            Self::MinPose => Self::min_distance_to_any_of(p, &inconsistent),
            Self::BoundingBoxArea => {
                let wh = p.map.width_height_meters();
                wh[0] * wh[1]
            }
        }
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

fn reversed_weights(weights: HashHistogram<usize, f64>) -> HashHistogram<usize, f64> {
    let max_weight = weights
        .iter()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(Ordering::Equal))
        .unwrap()
        .1;
    weights
        .iter()
        .map(|(i, w)| (*i, *max_weight - *w + 1.0))
        .collect()
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParticleFilterSettings {
    pub noises: Noises,
    pub num_particles: usize,
    pub square_size_m: f64,
    pub robot_radius_m: f64,
    pub selection_strategy: SelectionStrategy,
    pub weight_strategy: WeightStrategy,
    pub save_inputs: bool,
}

impl Default for ParticleFilterSettings {
    fn default() -> Self {
        Self {
            noises: Noises {
                clear: Noise {
                    stdev_x_y: 7e-4,
                    stdev_angle: Degrees::new(2e-4),
                },
                obst: Noise {
                    stdev_x_y: 0.032,
                    stdev_angle: Degrees::new(0.62),
                },
            },
            num_particles: 1000,
            square_size_m: 0.1,
            robot_radius_m: 0.2032,
            selection_strategy: SelectionStrategy::RankProportion,
            weight_strategy: WeightStrategy::MinPose,
            save_inputs: false,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum MapInput {
    Pose(RobotPose<Radians>),
    Collision(f64, Radians),
    RangeObject(f64, Radians),
}

impl MapInput {
    pub fn pose(&self) -> Option<RobotPose<Radians>> {
        if let Self::Pose(pose) = self {
            Some(*pose)
        } else {
            None
        }
    }

    pub fn collision(&self) -> bool {
        if let Self::Collision(_, _) = self {
            true
        } else {
            false
        }
    }

    pub fn obstacle(&self) -> Option<(f64, Radians)> {
        match self {
            Self::RangeObject(distance, heading) => Some((*distance, *heading)),
            Self::Collision(distance, heading) => Some((*distance, *heading)),
            Self::Pose(_) => None
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct ParticleFilter {
    last_raw: Option<RobotPose<Radians>>,
    particles: Vec<Particle>,
    total_iterations: usize,
    stats: BitGridStats,
    example_failure: Option<Particle>,
    selection_strategy: SelectionStrategy,
    weight_strategy: WeightStrategy,
    save_inputs: bool,
    inputs: Vec<MapInput>,
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
            save_inputs: settings.save_inputs,
            inputs: vec![],
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

    pub fn inputs(&self) -> Option<Vec<MapInput>> {
        if self.save_inputs {
            Some(self.inputs.clone())
        } else {
            None
        }
    }

    pub fn iterate(&mut self, map_input: MapInput) {
        if self.save_inputs {
            self.inputs.push(map_input);
        }
        if let Some(new_raw_pose) = map_input.pose() {
            self.last_raw = Some(new_raw_pose);
        }
        self.total_iterations += 1;
        self.update_all_particles(map_input);
        let consistent = self.find_consistent_particles();
        if consistent.len() == 0 {
            self.example_failure = Some(self.particles[0].clone());
        } else if consistent.len() < self.particles.len() {
            self.repopulate(consistent, map_input.collision());
        }
    }

    fn obstacle_point(&self, map_input: MapInput) -> Option<FloatPoint> {
        map_input
            .obstacle()
            .zip(self.last_raw)
            .map(|((distance, angle_offset), last_pose)| {
                let heading = last_pose.theta + angle_offset;
                last_pose.pos + (distance, heading).into()
            })
    }

    fn update_all_particles(&mut self, map_input: MapInput) {
        let obstacle = self.obstacle_point(map_input);
        let new_raw_pose = map_input.pose();
        for particle in self.particles.iter_mut() {
            if let Some(raw_pose) = new_raw_pose {
                particle.estimate.updated_raw_pose(raw_pose);
                particle
                    .map
                    .add_odometry_reading(&particle.estimated_pose().pos);
            }
            particle.add_noise(map_input.collision());
            particle.add_sensed_obstacles(obstacle);
        }
    }

    fn find_consistent_particles(&mut self) -> BitArray {
        (0..self.particles.len())
            .filter(|i| self.particles[*i].map.is_consistent())
            .collect()
    }

    fn repopulate(&mut self, consistent: BitArray, collision: bool) {
        let num_particles = self.particles.len();
        let ones = BitArray::ones(num_particles);
        let inconsistent = (&consistent ^ &ones)
            .iter()
            .inspect(|i| {
                self.stats
                    .gather_data_from(self.total_iterations, &self.particles[*i].map)
            })
            .map(|i| self.particles[i].clone())
            .collect::<Vec<_>>();

        self.particles = consistent
            .iter()
            .map(|i| self.particles[i].clone())
            .collect::<Vec<_>>();
        let selector = self.make_selector(&inconsistent);

        while self.particles.len() < num_particles {
            let choice = selector.choose();
            let mut new_particle = self.particles[choice].clone();
            new_particle.add_noise(collision);
            self.particles.push(new_particle);
        }
    }

    fn make_selector(&mut self, inconsistent: &Vec<Particle>) -> WalkerAliasTable {
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

#[derive(Copy, Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
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

#[derive(Copy, Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
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

    pub fn convert_to_raw_space(&self, estimate_space: &FloatPoint) -> FloatPoint {
        match self.last_raw {
            None => *estimate_space,
            Some(last_raw) => *estimate_space + last_raw.pos - self.current_estimate.pos,
        }
    }

    pub fn add_noise(&mut self, noise: Noise) {
        self.current_estimate = noise.noise(self.current_estimate);
    }
}

#[cfg(test)]
mod tests {
    use crate::{Degrees, FloatPoint, PoseEstimate, Radians, RobotPose};
    use std::f64::consts::PI;

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
    fn test_estimated_to_raw() {
        let example = PoseEstimate {
            last_raw: Some(RobotPose {
                pos: pt!(1.0, 2.0),
                theta: Radians::new(PI / 2.0),
            }),
            current_estimate: RobotPose {
                pos: pt!(1.25, 1.75),
                theta: Radians::new(PI / 2.0),
            },
        };
        for (other_estimate, expected) in [
            (pt!(3.0, 2.0), pt!(2.75, 2.25)),
            (pt!(-1.0, 1.0), pt!(-1.25, 1.25)),
        ] {
            assert_eq!(example.convert_to_raw_space(&other_estimate), expected);
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
