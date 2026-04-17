use std::{
    cmp::{Reverse, max, min},
    collections::{HashMap, VecDeque},
    f64::consts::PI,
};

use crate::{
    angle::Radians,
    bit_grid::BitGrid,
    point::{GridPoint, manhattan_offsets},
    pose::RobotPose,
    pt,
};
use priority_queue::PriorityQueue;

use crate::{BitGridMap, Particle};

#[derive(Clone, Default)]
pub struct PathsBackTo {
    start: GridPoint,
    parent_of: HashMap<GridPoint, Option<GridPoint>>,
    leaves: BitGrid,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum WhenToStop {
    First,
    All,
}

impl PathsBackTo {
    pub fn all(map: &BitGridMap, start: RobotPose<Radians>) -> PathsBackTo {
        Self::new(map, start, WhenToStop::All)
    }

    pub fn any(map: &BitGridMap, start: RobotPose<Radians>) -> PathsBackTo {
        Self::new(map, start, WhenToStop::First)
    }

    pub fn done(particle: &Particle) -> bool {
        let pbt = Self::any(&particle.map, particle.estimated_pose());
        pbt.no_path_to_unvisited()
    }

    fn new(map: &BitGridMap, start: RobotPose<Radians>, stop: WhenToStop) -> Self {
        let mut result = PathsBackTo::default();
        if map.is_consistent() {
            result.exhaustive_search(map, start, stop);
        }
        result
    }

    fn exhaustive_search(&mut self, map: &BitGridMap, start: RobotPose<Radians>, stop: WhenToStop) {
        let grid_step = (map.robot_shadow(start).width() / 2) as i64;
        let unvisited = map.unvisited();
        let mut queue = PriorityQueue::new();
        let start = GridVector::new(map, start);
        queue.push(start, Reverse(0));
        /*
        let (start, all_start_vecs) = GridVector::all_starts(map, start);
        self.start = start;
        for start_vec in all_start_vecs {
            queue.push(start_vec, Reverse(0));
        }
        */
        while let Some((current, cost)) = queue.pop() {
            if !self.parent_of.contains_key(&current.current) && current.clear_path(map) {
                self.add_vector(&current);
                if unvisited.contains(&current.current) && stop == WhenToStop::First {
                    break;
                }
                if map.all_spaces().contains(&current.current) {
                    for (successor, upcharge) in current.successors(grid_step) {
                        queue.push(successor, Reverse(cost.0 + upcharge));
                    }
                }
            }
        }
        self.leaves = &self.leaves & &unvisited;
    }

    fn add_vector(&mut self, v: &GridVector) {
        self.parent_of.insert(v.current, v.parent());
        self.leaves.insert(v.current);
        if let Some(parent) = v.parent().as_ref() {
            self.leaves.remove(parent);
        }
    }

    pub fn no_path_to_unvisited(&self) -> bool {
        self.leaves.len() == 0
    }

    pub fn shortest_path(&self) -> Option<VecDeque<GridPoint>> {
        let mut result = None;
        for leaf in self.leaves.iter() {
            let path_back = self.path_to_start(leaf);
            match result.as_mut() {
                None => result = Some(path_back),
                Some(best) => {
                    if path_back.len() < best.len() {
                        *best = path_back;
                    }
                }
            }
        }
        result
    }

    pub fn path_to_start(&self, leaf: GridPoint) -> VecDeque<GridPoint> {
        let mut path_back = VecDeque::new();
        path_back.push_front(leaf);
        while let Some(parent) = path_back
            .front()
            .and_then(|path| self.parent_of.get(path))
            .and_then(|parent| *parent)
        {
            path_back.push_front(parent);
        }
        path_back
    }

    pub fn leaves(&self) -> &BitGrid {
        &self.leaves
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
struct GridVector {
    is_start: bool,
    prev: GridPoint,
    current: GridPoint,
}

impl GridVector {
    fn new(map: &BitGridMap, pose: RobotPose<Radians>) -> Self {
        let heading = pose.theta;
        let tolerance = Radians::new(PI / 4.0);
        let current = map.to_point(pose.pos);
        let prev = current
            - if heading.abs() < tolerance {
                pt!(1, 0)
            } else if (heading - Radians::new(PI / 2.0)).abs() < tolerance {
                pt!(0, 1)
            } else if (heading - Radians::new(3.0 * PI / 2.0)).abs() < tolerance {
                pt!(0, -1)
            } else {
                pt!(-1, 0)
            };
        Self {
            is_start: true,
            prev,
            current,
        }
    }

    fn all_starts(map: &BitGridMap, pose: RobotPose<Radians>) -> (GridPoint, Vec<Self>) {
        let current = map.to_point(pose.pos);
        (current, current.manhattan_neighbors().map(|prev| Self {is_start: true, prev, current}).collect())
    }

    fn parent(&self) -> Option<GridPoint> {
        if self.is_start { None } else { Some(self.prev) }
    }

    fn clear_path(&self, map: &BitGridMap) -> bool {
        if self.horizontal() {
            (min(self.current[0], self.prev[0])..=max(self.current[0], self.prev[0]))
                .all(|i| !map.shadow_envelops_obstacle(pt!(i, self.current[1])))
        } else if self.vertical() {
            (min(self.current[1], self.prev[1])..=max(self.current[1], self.prev[1]))
                .all(|i| !map.shadow_envelops_obstacle(pt!(self.current[0], i)))
        } else {
            false
        }
    }

    fn successors(&self, grid_step: i64) -> impl Iterator<Item = (Self, u64)> {
        let copy = *self;
        manhattan_offsets().map(move |offset| {
            let next = copy.current + offset * grid_step;
            let cost = copy.num_moves_needed(next);
            (
                Self {
                    is_start: false,
                    prev: copy.current,
                    current: next,
                },
                cost,
            )
        })
    }

    fn horizontal(&self) -> bool {
        self.prev[1] == self.current[1]
    }

    fn vertical(&self) -> bool {
        self.prev[0] == self.current[0]
    }

    fn num_moves_needed(&self, next: GridPoint) -> u64 {
        if self.horizontal() && next[1] == self.current[1]
            || self.vertical() && next[0] == self.current[0]
        {
            if next == self.prev { 3 } else { 1 }
        } else {
            2
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{BitGridMap, angle::Radians, path_plan::PathsBackTo, pose::RobotPose};

    const TEST_MAP_STR_1: &str = r#"{"obstacles":{"bits":{"bits":[1]},"bounds":{"min":{"coords":[-15,3]},"max":{"coords":[-15,3]}}},"spaces":{"bits":{"bits":[8935143584848674816,13835058055281115134,206156595199]},"bounds":{"min":{"coords":[-18,-5]},"max":{"coords":[2,2]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_STR_1: &str = r#"{"pos":{"coords":[-1.5637336449019554,0.11068795293575091]},"theta":-3.074780485700006}"#;

    const TEST_MAP_STR_2: &str = r#"{"obstacles":{"bits":{"bits":[1]},"bounds":{"min":{"coords":[-15,3]},"max":{"coords":[-15,3]}}},"spaces":{"bits":{"bits":[14951951243906514944,18374686479671558143,805292031]},"bounds":{"min":{"coords":[-17,-5]},"max":{"coords":[2,2]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_STR_2: &str = r#"{"pos":{"coords":[-1.5866413378378004,0.0998764804308733]},"theta":-3.086761081685883}"#;

    #[test]
    fn test_unexpected_no_paths() {
        let paths = paths_from(TEST_MAP_STR_1, TEST_POSE_STR_1);
        println!("{}", paths.leaves);
    }

    #[test]
    fn test_expected_paths() {
        let paths = paths_from(TEST_MAP_STR_2, TEST_POSE_STR_2);
        println!("{}", paths.leaves);
    }

    fn paths_from(map: &str, pose: &str) -> PathsBackTo {
        let map: BitGridMap = serde_json::from_str(map).unwrap();
        println!("Consistent? {}", map.is_consistent());
        print!("Obstacles:");
        for obstacle in map.all_obstacles().iter() {
            print!(" {obstacle} ({}) ", map.num_neighbors_spaces(&obstacle));
        }
        println!();
        let start: RobotPose<Radians> = serde_json::from_str(pose).unwrap();
        println!("Robot at: {}", map.to_point(start.pos));
        print!("Shadow:");
        for bot in map.robot_shadow(start).iter() {
            print!(" {bot}");
        }
        println!();
        PathsBackTo::any(&map, start)
    }
}