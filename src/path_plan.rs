use std::{
    cmp::{Reverse, max, min},
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
        let grid_step = map.robot_shadow(start).width() / 2;
        let start = GridVector::new(map, start);
        self.start = start.current;
        let unvisited = map.unvisited();
        let mut queue = PriorityQueue::new();
        queue.push(start, Reverse(0));
        while let Some((current, cost)) = queue.pop() {
            if !self.parent_of.contains_key(&current.current) && current.clear_path(map) {
                self.add_vector(&current);
                if unvisited.get(&current.current) && stop == WhenToStop::First {
                    break;
                }
                if map.all_spaces().get(&current.current) {
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
        self.leaves.set(v.current, true);
        if let Some(parent) = v.parent() {
            self.leaves.set(parent, false);
        }
    }

    pub fn no_path_to_unvisited(&self) -> bool {
        self.leaves.count_ones() == 0
    }

    pub fn shortest_path(&self) -> Option<VecDeque<GridPoint>> {
        let mut result = None;
        for leaf in self.leaves.ones() {
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

    fn clear_path(&self, map: &BitGridMap) -> bool {
        if self.horizontal() {
            (min(self.current[0], self.prev[0])..=max(self.current[0], self.prev[0]))
                .all(|i| !map.collides_at_position(pt!(i, self.current[1])))
        } else if self.vertical() {
            (min(self.current[1], self.prev[1])..=max(self.current[1], self.prev[1]))
                .all(|i| !map.collides_at_position(pt!(self.current[0], i)))
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
