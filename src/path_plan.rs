use std::{
    cmp::Reverse,
    collections::{HashMap, VecDeque},
    f64::consts::PI,
};

use bit_grid::{
    BitGrid,
    angle::Radians,
    point::{GridPoint, Point, manhattan_offsets},
    pose::RobotPose,
    pt,
};
use priority_queue::PriorityQueue;

use crate::BitGridMap;

pub fn waypoint_grid(map: &BitGridMap, start: RobotPose<Radians>) -> BitGrid {
    let step_size = map.robot_shadow(start).width() / 2;
    let mut waypoints = BitGrid::default();
    let mut queue = VecDeque::new();
    queue.push_back(map.to_point(start.pos));
    while let Some(current) = queue.pop_front() {
        if !waypoints.get(&current)
            && (&map.grid_shadow(current) & &map.all_obstacles()).count_ones() == 0
        {
            waypoints.set(current, true);
            if map.all_spaces().get(&current) {
                for offset in manhattan_offsets() {
                    queue.push_back(current + offset * step_size);
                }
            }
        }
    }
    waypoints
}

pub fn paths_from(map: &BitGridMap, start: RobotPose<Radians>) -> PathsBackTo {
    let grid_step = map.robot_shadow(start).width() / 2;
    let mut result = PathsBackTo::default();
    let start = GridVector::new(map, start);
    result.start = start.current;
    let mut queue = PriorityQueue::new();
    queue.push(start, Reverse(0));
    while let Some((current, cost)) = queue.pop() {
        if !result.parent_of.contains_key(&current.current)
            && (&map.grid_shadow(current.current) & &map.all_obstacles()).count_ones() == 0
        {
            result.parent_of.insert(current.current, current.parent());
            result.leaves.set(current.current, true);
            if let Some(parent) = current.parent() {
                result.leaves.set(parent, false);
            }
            if map.all_spaces().get(&current.current) {
                for (successor, upcharge) in current.successors(grid_step) {
                    queue.push(successor, Reverse(cost.0 + upcharge));
                }
            }
        }
    }
    result.leaves = &result.leaves & &map.unvisited();
    result
}

#[derive(Clone, Default)]
pub struct PathsBackTo {
    start: GridPoint,
    parent_of: HashMap<GridPoint, Option<GridPoint>>,
    leaves: BitGrid,
}

impl PathsBackTo {
    pub fn shortest_path(&self) -> Option<VecDeque<GridPoint>> {
        let mut result = None;
        for leaf in self.leaves.ones() {
            let mut path_back = VecDeque::new();
            path_back.push_front(leaf);
            while let Some(parent) = path_back
                .front()
                .and_then(|path| self.parent_of.get(path))
                .and_then(|parent| *parent)
            {
                path_back.push_front(parent);
            }
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
}

#[derive(Copy, Clone, PartialEq, Eq, Hash)]
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

    fn parent(&self) -> Option<GridPoint> {
        if self.is_start { None } else { Some(self.prev) }
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
