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

    pub fn shortest_path_points(map: &BitGridMap, start: RobotPose<Radians>) -> BitGrid {
        Self::any(map, start).shortest_path().map_or(BitGrid::default(), |shortest| shortest.iter().collect())        
    }

    pub fn all_path_points(map: &BitGridMap, start: RobotPose<Radians>) -> BitGrid {
        let paths_back = Self::all(map, start);
        let mut grid = BitGrid::default();
        for leaf in paths_back.leaves().iter() {
            for square in paths_back.path_to_start(leaf) {
                grid.insert(square);
            }
        }
        grid
    }

    pub fn path_points_to(map: &BitGridMap, start: RobotPose<Radians>, target: GridPoint) -> BitGrid {
        Self::all(map, start).path_to_start(target).iter().collect()
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

pub fn necessary_turns_from<I: Iterator<Item=GridPoint>>(path: I, map: &BitGridMap) -> Vec<GridPoint> {
    let mut result = vec![];
    for point in path {
        if result.len() == 0 || !map.clear_path_between(&result[result.len() - 1], &point) {
            result.push(point);
        }
    }
    result
}

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
struct GridVector {
    is_start: bool,
    prev: GridPoint,
    current: GridPoint,
}

impl GridVector {
    fn new(map: &BitGridMap, pose: RobotPose<Radians>) -> Self {
        let current = map.to_point(pose.pos);
        let prev = current - heading2manhattan(pose.theta);
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

fn heading2manhattan(heading: Radians) -> GridPoint {
    let tolerance = Radians::new(PI / 4.0);
    if heading.abs() < tolerance {
        pt!(1, 0)
    } else if (heading - Radians::new(PI / 2.0)).abs() < tolerance {
        pt!(0, 1)
    } else if (heading - Radians::new(3.0 * PI / 2.0)).abs() < tolerance {
        pt!(0, -1)
    } else {
        pt!(-1, 0)
    }
}

#[cfg(test)]
mod tests {
    use crate::{BitGridMap, angle::Radians, bit_grid::BitGrid, path_plan::{PathsBackTo, necessary_turns_from}, pose::RobotPose, pt};

    const TEST_MAP_STR_1: &str = r#"{"obstacles":{"bits":{"bits":[1]},"bounds":{"min":{"coords":[-15,3]},"max":{"coords":[-15,3]}}},"spaces":{"bits":{"bits":[8935143584848674816,13835058055281115134,206156595199]},"bounds":{"min":{"coords":[-18,-5]},"max":{"coords":[2,2]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_STR_1: &str = r#"{"pos":{"coords":[-1.5637336449019554,0.11068795293575091]},"theta":-3.074780485700006}"#;

    const TEST_MAP_STR_2: &str = r#"{"obstacles":{"bits":{"bits":[1]},"bounds":{"min":{"coords":[-15,3]},"max":{"coords":[-15,3]}}},"spaces":{"bits":{"bits":[14951951243906514944,18374686479671558143,805292031]},"bounds":{"min":{"coords":[-17,-5]},"max":{"coords":[2,2]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_STR_2: &str =
        r#"{"pos":{"coords":[-1.5866413378378004,0.0998764804308733]},"theta":-3.086761081685883}"#;

    const TEST_MAP_316: &str = r#"{"obstacles":{"bits":{"bits":[216172783188049920,35701915648,549755813921,18014398509481984,274877923328,2251799813718016,72057594037927936,0,0,9223372036854775808,0,2305843009213693952,8796093022210,536870912,162160372911964208]},"bounds":{"min":{"coords":[-15,-21]},"max":{"coords":[24,2]}}},"spaces":{"bits":{"bits":[9223372036984799232,9006312343995391,17293259623443853056,1125075206013439,18446726482060247024,17870424058894221375,18445618105116786175,18158654434844835855,18446743938451636223,18176105883602192143,18446744009284517887,18412967076504265215,18446744057066553343,13835620902156329087,18311631686976081919,70300827910175,2305843009219952671,0]},"bounds":{"min":{"coords":[-16,-22]},"max":{"coords":[25,3]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_316: &str = r#"{"pos":{"coords":[-0.6696478960737573,-1.5008256983736394]},"theta":-1.9513766899243616}"#;

    const TEST_MAP_316_STOPPED: &str = r#"{"obstacles":{"bits":{"bits":[1]},"bounds":{"min":{"coords":[1,37]},"max":{"coords":[1,37]}}},"spaces":{"bits":{"bits":[2296835809958820988,1135999956104789535,9782911240656141855,9782911240656142095,9782911240652070671,126647214570086159,1]},"bounds":{"min":{"coords":[-6,-4]},"max":{"coords":[2,38]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;
    const TEST_POSE_316_STOPPED: &str = r#"{"pos":{"coords":[0.09699651483408887,3.717113913552248]},"theta":1.2385537322981237}"#;

    #[test]
    fn test_unexpected_no_paths() {
        let paths = paths_from(TEST_MAP_STR_1, TEST_POSE_STR_1);
        println!("leaves: {}", paths.leaves);
    }

    #[test]
    fn test_expected_paths() {
        let paths = paths_from(TEST_MAP_STR_2, TEST_POSE_STR_2);
        println!("{}", paths.leaves);
    }

    #[test]
    fn test_unexpected_no_paths_316() {
        let paths = paths_from(TEST_MAP_316_STOPPED, TEST_POSE_316_STOPPED);
        println!("leaves: {}", paths.leaves);
    }

    #[test]
    fn test_316_shortest() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        let pose: RobotPose<Radians> = serde_json::from_str(TEST_POSE_316).unwrap();
        let shortest = PathsBackTo::shortest_path_points(&map, pose);
        println!("{}", map.map_pose_path_str(Some(pose), shortest));
    }

    #[test]
    fn test_316_all() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        let pose: RobotPose<Radians> = serde_json::from_str(TEST_POSE_316).unwrap();
        let on_grid = map.to_point(pose.pos);
        println!("Pose on grid: {on_grid}");
        let shortest = PathsBackTo::all_path_points(&map, pose);
        println!("{}", map.map_pose_path_str(Some(pose), shortest));
    }

    #[test]
    fn test_316_specific() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        let pose: RobotPose<Radians> = serde_json::from_str(TEST_POSE_316).unwrap();
        let shortest = PathsBackTo::path_points_to(&map, pose, pt!(6, -23));
        println!("{}", map.map_pose_path_str(Some(pose), shortest));
    }

    #[test]
    fn test_316_bypass() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        let pose: RobotPose<Radians> = serde_json::from_str(TEST_POSE_316).unwrap();
        let path = necessary_turns_from(PathsBackTo::all(&map, pose).path_to_start(pt!(6, -23)).iter().copied(), &map);
        println!("{path:?}");
        let path_points = path.iter().collect::<BitGrid>();
        println!("{}", map.map_pose_path_str(Some(pose), path_points));
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

    #[test]
    fn test_necessary_turns_from() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        let pose: RobotPose<Radians> = serde_json::from_str(TEST_POSE_316).unwrap();
        for (end, expected_path) in [
            (pt!(18,  -9),  vec![pt!(-6, -15), pt!(-4, -9), pt!(18,  -9)]),
            (pt!(18, -19),  vec![pt!(-6, -15), pt!(-4, -9), pt!(18, -19)]),
            (pt!( 6, -23),  vec![pt!(-6, -15), pt!(-4, -9), pt!( 6, -17), pt!(6, -23)])
        ] {
            let path = necessary_turns_from(PathsBackTo::all(&map, pose).path_to_start(end).iter().copied(), &map);
            assert_eq!(expected_path, path);    
        }
    }
}
