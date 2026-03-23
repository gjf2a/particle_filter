use std::{collections::{HashMap, VecDeque}, f64::consts::PI};

use bit_grid::{BitGrid, angle::Radians, point::{GridPoint, Point}, pose::RobotPose, pt};

use crate::{BitGridMap, Particle};

pub fn waypoint_grid(map: &BitGridMap, start: RobotPose<Radians>) -> BitGrid {
    let step_size = map.robot_shadow(start).width() / 2;
    let mut waypoints = BitGrid::default();
    let mut queue = VecDeque::new();
    queue.push_back(map.to_point(start.pos));
    while let Some(current) = queue.pop_front() {
        if !waypoints.get(&current) && (&map.grid_shadow(current) & &map.all_obstacles()).count_ones() == 0 {
            waypoints.set(current, true);
            if map.all_spaces().get(&current) {
                for offset in [pt!(step_size, 0), pt!(-step_size, 0), pt!(0, step_size), pt!(0, -step_size)] {
                    queue.push_back(current + offset);
                }
            }
        }
    }
    waypoints
}

struct GridVector {
    prev: GridPoint,
    current: GridPoint,
}

impl GridVector {
    fn new(particle: &Particle) -> Self {
        let heading = particle.estimated_pose().theta;
        let tolerance = Radians::new(PI / 4.0); 
        let current = particle.map.to_point(particle.estimated_pose().pos);
        let prev = current - if heading.abs() < tolerance {
            pt!(1, 0)
        } else if (heading - Radians::new(PI / 2.0)).abs() < tolerance {
            pt!(0, 1)
        } else if (heading - Radians::new(3.0 * PI / 2.0)).abs() < tolerance {
            pt!(0, -1)
        } else {
            pt!(-1, 0)
        };
        Self {prev, current}
    }

    fn horizontal(&self) -> bool {
        self.prev[1] == self.current[1]
    }

    fn vertical(&self) -> bool {
        self.prev[0] == self.current[0]
    }

    fn num_moves_needed(&self, next: GridPoint) -> u64 {
        if self.horizontal() && next[1] == self.current[1] || self.vertical() && next[0] == self.current[0] {
            if next == self.prev {
                3
            } else {
                1
            }
        } else {
            2
        }
    }
}

#[derive(Clone, Default)]
pub struct PathsBackTo {
    start: GridPoint,
    parent_of: HashMap<GridPoint, Option<GridPoint>>,
    leaves: BitGrid,
}

pub fn paths_from(particle: &Particle) -> PathsBackTo {
    let result = PathsBackTo::default();
    let start = particle.map.to_point(particle.estimated_pose().pos);
    
    result
}