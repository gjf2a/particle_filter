use std::collections::{HashMap, VecDeque};

use crate::StatCollector;
use bit_grid::{
    BitGrid, ColumnMajorCoordIter,
    angle::Radians,
    point::{BoundingBox, FloatPoint, GridPoint, Point},
    pose::RobotPose,
    pt, span,
};
use enum_iterator::{Sequence, all};
use hash_histogram::HashHistogram;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum Cell {
    Obstacle,
    Space,
    Unvisited,
    Inconsistent,
}

fn to_square(square_size_m: f64, value_meters: f64) -> i64 {
    (value_meters / square_size_m) as i64
}

fn to_meters(square_size_m: f64, value_squares: i64) -> f64 {
    value_squares as f64 * square_size_m
}

fn to_float_point(square_size_m: f64, gp: GridPoint) -> FloatPoint {
    gp.iter().map(|g| to_meters(square_size_m, g)).collect()
}

fn to_grid_point(square_size_m: f64, fp: FloatPoint) -> GridPoint {
    fp.iter().map(|f| to_square(square_size_m, f)).collect()
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct BitGridMap {
    obstacles: BitGrid,
    spaces: BitGrid,
    shadow: BitGrid,
    square_size_m: f64,
    brand_new: bool,
    space_contiguous: bool,
}

impl BitGridMap {
    fn create_shadow(square_size_m: f64, robot_radius_m: f64) -> BitGrid {
        let grid_radius = to_square(square_size_m, robot_radius_m);
        let grid_diameter = grid_radius * 2 + 1;
        let mut shadow = BitGrid::default();
        for coord in
            ColumnMajorCoordIter::new(-grid_radius, -grid_radius, grid_diameter, grid_diameter)
        {
            let float = to_float_point(square_size_m, coord);
            if float.euclidean_distance(pt!(0.0, 0.0)) < robot_radius_m {
                shadow.set(coord, true);
            }
        }
        shadow
    }

    pub fn new(square_size_m: f64, robot_radius_m: f64) -> Self {
        Self {
            obstacles: BitGrid::default(),
            spaces: BitGrid::default(),
            shadow: Self::create_shadow(square_size_m, robot_radius_m),
            square_size_m,
            brand_new: true,
            space_contiguous: true,
        }
    }

    pub fn is_consistent(&self) -> bool {
        self.space_contiguous && self.obstacle_space_independent()
    }

    pub fn add_obstacle_at(&mut self, obstacle: &FloatPoint) {
        let p = self.to_point(*obstacle);
        self.obstacles.set(p, true);
    }

    pub fn add_odometry_reading(&mut self, odometry_location: &FloatPoint) {
        let overlap = self.draw_overlapping_shadow_on(self.to_point(*odometry_location));
        self.space_contiguous = self.space_contiguous && (self.brand_new || overlap);
        self.brand_new = false;
    }

    pub fn map_words_used(&self) -> usize {
        self.obstacles.words_used() + self.spaces.words_used()
    }

    pub fn width_height_meters(&self) -> FloatPoint {
        FloatPoint::new([
            self.width() as f64 * self.square_size_m,
            self.height() as f64 * self.square_size_m,
        ])
    }

    pub fn points(&self) -> impl Iterator<Item = (GridPoint, Cell)> {
        self.spaces.coord_iter().map(|p| (p, self.cell_for(&p)))
    }

    pub fn cell_for(&self, p: &GridPoint) -> Cell {
        if self.spaces.get(p) {
            if self.obstacles.get(p) {
                if self.consistent_obstacle(p) {
                    Cell::Obstacle
                } else {
                    Cell::Inconsistent
                }
            } else {
                Cell::Space
            }
        } else if self.obstacles.get(p) {
            Cell::Obstacle
        } else {
            Cell::Unvisited
        }
    }

    pub fn bounding_box(&self) -> BoundingBox<i64> {
        self.spaces.bounding_box() | self.obstacles.bounding_box()
    }

    pub fn bordered_bounding_box(&self) -> BoundingBox<i64> {
        let mut result = self.bounding_box();
        result.grow(self.shadow.width());
        result
    }

    pub fn width(&self) -> i64 {
        self.bounding_box().width()
    }

    pub fn height(&self) -> i64 {
        self.bounding_box().height()
    }

    fn to_point(&self, fp: FloatPoint) -> GridPoint {
        to_grid_point(self.square_size_m, fp)
    }

    pub fn robot_shadow(&self, pose: RobotPose<Radians>) -> BitGrid {
        self.grid_shadow(self.to_point(pose.pos))
    }

    fn grid_shadow(&self, grid_point: GridPoint) -> BitGrid {
        self.shadow.translated(grid_point)
    }

    fn draw_overlapping_shadow_on(&mut self, grid_point: GridPoint) -> bool {
        let mut overlapping = false;
        for p in self.grid_shadow(grid_point).ones() {
            overlapping |= self.spaces.get(&p);
            self.spaces.set(p, true);
        }
        overlapping
    }

    pub fn num_obstacles(&self) -> usize {
        self.obstacles.count_ones()
    }

    pub fn num_spaces(&self) -> usize {
        self.spaces.count_ones()
    }

    pub fn space_contiguous(&self) -> bool {
        self.space_contiguous
    }

    pub fn obstacle_space_independent(&self) -> bool {
        self.obstacles.ones().all(|p| self.consistent_obstacle(&p))
    }

    pub fn num_neighbors_spaces(&self, p: &GridPoint) -> usize {
        self.spaces
            .manhattan_neighbors(p)
            .filter(|(_, is_on)| *is_on)
            .count()
    }

    pub fn consistent_obstacle(&self, p: &GridPoint) -> bool {
        let neighbor_spaces = self.num_neighbors_spaces(p);
        0 < neighbor_spaces && neighbor_spaces < 4
    }

    pub fn inconsistency(&self) -> Option<Inconsistency> {
        if !self.space_contiguous {
            Some(Inconsistency::SeparatedSpaces)
        } else if !self.obstacle_space_independent() {
            Some(Inconsistency::ObstacleSpaceOverlap)
        } else {
            None
        }
    }

    pub fn all_spaces(&self) -> &BitGrid {
        &self.spaces
    }

    pub fn all_obstacles(&self) -> &BitGrid {
        &self.obstacles
    }

    pub fn all_visited(&self) -> BitGrid {
        &self.spaces | &self.obstacles
    }

    pub fn unvisited(&self) -> BitGrid {
        let both = self.all_visited();
        &(BitGrid::one_grid(self.bordered_bounding_box())) ^ &both
    }

    pub fn all_frontier_spaces(&self) -> BitGrid {
        let spaces_with_obstacles = self.all_visited();
        spaces_with_obstacles
            .ones_touching_zeros()
            .filter(|p| !self.obstacles.get(p))
            .collect()
    }

    pub fn open_frontier_spaces(&self) -> BitGrid {
        self.all_frontier_spaces()
            .ones()
            .filter(|p| {
                let shadow = self.grid_shadow(*p);
                (&shadow & &self.obstacles).count_ones() == 0
            })
            .collect()
    }

    pub fn waypoint_grid(&self, start: RobotPose<Radians>) -> BitGrid {
        let step_size = self.shadow.width() / 2;
        let mut waypoints = BitGrid::default();
        let mut queue = VecDeque::new();
        queue.push_back(self.to_point(start.pos));
        while let Some(current) = queue.pop_front() {
            if !waypoints.get(&current) && (&self.grid_shadow(current) & &self.obstacles).count_ones() == 0 {
                waypoints.set(current, true);
                if self.spaces.get(&current) {
                    for offset in [pt!(step_size, 0), pt!(-step_size, 0), pt!(0, step_size), pt!(0, -step_size)] {
                        queue.push_back(current + offset);
                    }
                }
            }
        }
        waypoints
    }
}

impl StatCollector<BitGridMap> for BitGridStats {
    fn gather_data_from(&mut self, iteration: usize, particle: &BitGridMap) {
        if let Some(inconsistency) = particle.inconsistency() {
            self.stats.get_mut(&inconsistency).unwrap().bump(&iteration);
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Sequence, Debug, Serialize, Deserialize)]
pub enum Inconsistency {
    ObstacleSpaceOverlap,
    SeparatedSpaces,
    OffMap,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct BitGridStats {
    pub stats: HashMap<Inconsistency, HashHistogram<usize, usize>>,
}

impl BitGridStats {
    pub fn stats_for(&self, key: &Inconsistency) -> &HashHistogram<usize, usize> {
        self.stats.get(key).unwrap()
    }

    pub fn total_for(&self, key: &Inconsistency) -> usize {
        self.stats_for(key).total_count()
    }

    pub fn total(&self) -> usize {
        all::<Inconsistency>().map(|inc| self.total_for(&inc)).sum()
    }

    pub fn by_iteration(&self) -> HashHistogram<usize, usize> {
        let mut result = HashHistogram::new();
        for counts in self.stats.values() {
            for (key, count) in counts.iter() {
                result.bump_by(key, *count);
            }
        }
        result
    }
}

impl Default for BitGridStats {
    fn default() -> Self {
        Self {
            stats: all::<Inconsistency>()
                .map(|inc| (inc, HashHistogram::default()))
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use bit_grid::{
        point::{GridPoint, Point},
        pt,
    };

    use crate::bit_grid_map::BitGridMap;

    #[test]
    fn test_shadow() {
        let mut tester = BitGridMap::new(0.1, 0.2032);
        let shadow = tester.grid_shadow(GridPoint::default());
        let expected = "(-2,-2)\n00100\n01110\n11111\n01110\n00100";
        let shadow_str = format!("{shadow}");
        assert_eq!(expected, shadow_str);

        tester.obstacles = [pt!(0, 0), pt!(-2, -1), pt!(-2, 0)].iter().collect();
        let intersected = &tester.obstacles & &shadow;
        assert_eq!(intersected.count_ones(), 2);

        let intersected = tester.obstacles.overlapping_counts(&shadow);
        assert_eq!(intersected, 2);
    }
}
