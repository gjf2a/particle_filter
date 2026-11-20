use std::collections::{HashMap, VecDeque};

use crate::{
    BitGridMap,
    bit_grid::BitGrid,
    point::{GridOutlineIterator, GridPoint},
};

pub struct ClearPathGraph {
    distances: HashMap<GridPoint, HashMap<GridPoint, f64>>,
}

fn set_from_to<V>(
    map: &mut HashMap<GridPoint, HashMap<GridPoint, V>>,
    from: &GridPoint,
    to: &GridPoint,
    value: V,
) {
    match map.get_mut(from) {
        Some(to_values) => {
            to_values.insert(*to, value);
        }
        None => {
            let mut to_values = HashMap::new();
            to_values.insert(*to, value);
            map.insert(*from, to_values);
        }
    }
}

impl ClearPathGraph {
    pub fn corner_graph(map: &BitGridMap, max_path_overlap: usize) -> Self {
        let corners = map.all_corners().iter().collect::<Vec<_>>();
        Self::make_graph(&map, corners, max_path_overlap)
    }

    pub fn corner_current_graph(map: &BitGridMap, current: GridPoint, max_path_overlap: usize, max_current_overlap: usize) -> Option<Self> {
        find_clear_current(map, current, max_current_overlap).map(|current| {
            let mut corners = map.all_corners().iter().collect::<Vec<_>>();
            corners.push(current);
            Self::make_graph(&map, corners, max_path_overlap)
        })
    }

    pub fn corner_current_frontier_graph(map: &BitGridMap, current: GridPoint, max_path_overlap: usize, max_current_overlap: usize) -> Option<Self> {
        find_clear_current(map, current, max_current_overlap).map(|current| {
            let frontier = map.open_frontier_spaces();
            let mut corners = map.all_corners().iter().collect::<Vec<_>>();
            corners.push(current);
            for frontier_square in frontier.iter() {
                if !map.all_obstacles().contains(&frontier_square) {
                    corners.push(frontier_square)
                }
            }
            Self::make_graph(&map, corners, max_path_overlap)
        })
    }

    fn frontier_path<F: Fn(f64, f64) -> bool>(
        map: &BitGridMap,
        current: GridPoint,
        max_path_overlap: usize,
        max_current_overlap: usize,
        prefer_first_distance: F,
    ) -> Option<VecDeque<GridPoint>> {
        if let Some(cpg) = Self::corner_current_frontier_graph(map, current, max_path_overlap, max_current_overlap) {
            println!("Created ClearPathGraph object");
            let paths = ShortestPaths::new(&cpg);
            println!("Created ShortestPaths object");
            let frontier = map.open_frontier_spaces();
            println!("{} frontier spaces", frontier.len());
            let mut best = None;
            for candidate in frontier.iter() {
                if let Some(distance) = paths.distance_between(&current, &candidate) {
                    if best.map_or(true, |(best_distance, _)| {
                        prefer_first_distance(distance, best_distance)
                    }) {
                        best = Some((distance, candidate));
                    }
                }
            }
            best.map(|(_, frontier)| paths.path_between(&current, &frontier))
        } else {
            None
        }
    }

    pub fn shortest_frontier_path(
        map: &BitGridMap,
        current: GridPoint,
        max_path_overlap: usize,
        max_current_overlap: usize,
    ) -> Option<VecDeque<GridPoint>> {
        Self::frontier_path(map, current, max_path_overlap, max_current_overlap, |a, b| a < b)
    }

    pub fn longest_frontier_path(
        map: &BitGridMap,
        current: GridPoint,
        max_path_overlap: usize,
        max_current_overlap: usize,
    ) -> Option<VecDeque<GridPoint>> {
        Self::frontier_path(map, current, max_path_overlap, max_current_overlap, |a, b| a > b)
    }

    fn make_graph(map: &BitGridMap, vertices: Vec<GridPoint>, max_path_overlap: usize) -> Self {
        let mut distances = HashMap::new();
        for i in 0..vertices.len() {
            for j in (i + 1)..vertices.len() {
                if map.clear_path_between(&vertices[i], &vertices[j], max_path_overlap) {
                    let distance = vertices[i].euclidean_distance(vertices[j]);
                    set_from_to(&mut distances, &vertices[i], &vertices[j], distance);
                    set_from_to(&mut distances, &vertices[j], &vertices[i], distance);
                }
            }
        }
        assert!(distances.len() <= vertices.len());
        Self { distances }
    }

    pub fn edges_from(&self, p: &GridPoint) -> impl Iterator<Item = (&GridPoint, &f64)> {
        // From https://users.rust-lang.org/t/how-to-return-an-empty-iterator-if-my-vector-isnt-found/100248
        self.distances.get(p).into_iter().flatten()
    }

    pub fn vertices(&self) -> impl Iterator<Item = &GridPoint> {
        self.distances.keys()
    }

    pub fn is_connected(&self) -> bool {
        let vertices = self.vertices().collect::<BitGrid>();
        match vertices.iter().next() {
            None => true,
            Some(start) => {
                let mut visited = BitGrid::default();
                let mut pending = vec![start];
                while let Some(current) = pending.pop() {
                    if !visited.contains(&current) {
                        visited.insert(current);
                        for (neighbor, _) in self.edges_from(&current) {
                            pending.push(*neighbor);
                        }
                    }
                }
                visited == vertices
            }
        }
    }
}

fn find_clear_current(map: &BitGridMap, current: GridPoint, max_current_overlap: usize) -> Option<GridPoint> {
    for d in 1..=(1 + map.robot_radius_grid()) {
        for candidate in GridOutlineIterator::new(current, d) {
            let collisions = &map.grid_shadow(current) & map.all_obstacles();
            if collisions.len() <= max_current_overlap {
                return Some(candidate);
            }
        }
    }
    None
}

pub struct ShortestPaths {
    distances: HashMap<GridPoint, HashMap<GridPoint, f64>>,
    predecessors: HashMap<GridPoint, HashMap<GridPoint, GridPoint>>,
}

impl ShortestPaths {
    pub fn new(graph: &ClearPathGraph) -> Self {
        // This implementation of the Floyd-Warshall algorithm is derived from Chapter 22 of:
        // https://books.goalkicker.com/AlgorithmsBook/
        assert!(graph.is_connected());
        let mut distances = HashMap::new();
        let mut predecessors = HashMap::new();
        for n1 in graph.vertices() {
            set_from_to(&mut distances, n1, n1, 0.0);
            set_from_to(&mut predecessors, n1, n1, *n1);
            for (n2, cost) in graph.edges_from(n1) {
                set_from_to(&mut distances, n1, n2, *cost);
                set_from_to(&mut predecessors, n1, n2, *n1);
            }
        }
        for k in graph.vertices() {
            for i in graph.vertices() {
                for j in graph.vertices() {
                    if let Some(i2k) = distances.get(&i).unwrap().get(&k) {
                        if let Some(k2j) = distances.get(&k).unwrap().get(&j) {
                            let alt_i2j = i2k + k2j;
                            if distances
                                .get(&i)
                                .unwrap()
                                .get(&j)
                                .map_or(true, |i2j| *i2j > alt_i2j)
                            {
                                set_from_to(&mut distances, &i, &j, alt_i2j);
                                let k_pred =
                                    predecessors.get(&k).unwrap().get(&j).copied().unwrap();
                                set_from_to(&mut predecessors, &i, &j, k_pred);
                            }
                        }
                    }
                }
            }
        }
        Self {
            distances,
            predecessors,
        }
    }

    pub fn distance_between(&self, p1: &GridPoint, p2: &GridPoint) -> Option<f64> {
        self.distances.get(p1).and_then(|ds| ds.get(p2)).copied()
    }

    pub fn path_between(&self, p1: &GridPoint, p2: &GridPoint) -> VecDeque<GridPoint> {
        let mut result = VecDeque::new();
        let p1_preds = self.predecessors.get(&p1).unwrap();
        result.push_front(*p2);
        while &result[0] != p1 {
            result.push_front(p1_preds.get(&result[0]).copied().unwrap())
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use crate::{BitGridMap, clear_path_graph::ClearPathGraph};

    const TEST_MAP_316: &str = r#"{"obstacles":{"bits":{"bits":[216172783188049920,35701915648,549755813921,18014398509481984,274877923328,2251799813718016,72057594037927936,0,0,9223372036854775808,0,2305843009213693952,8796093022210,536870912,162160372911964208]},"bounds":{"min":{"coords":[-15,-21]},"max":{"coords":[24,2]}}},"spaces":{"bits":{"bits":[9223372036984799232,9006312343995391,17293259623443853056,1125075206013439,18446726482060247024,17870424058894221375,18445618105116786175,18158654434844835855,18446743938451636223,18176105883602192143,18446744009284517887,18412967076504265215,18446744057066553343,13835620902156329087,18311631686976081919,70300827910175,2305843009219952671,0]},"bounds":{"min":{"coords":[-16,-22]},"max":{"coords":[25,3]}}},"shadow":{"bits":{"bits":[4685252]},"bounds":{"min":{"coords":[-2,-2]},"max":{"coords":[2,2]}}},"square_size_m":0.1,"brand_new":false,"space_contiguous":true}"#;

    #[test]
    fn test_build_graph() {
        let map: BitGridMap = serde_json::from_str(TEST_MAP_316).unwrap();
        println!("About to build graph");
        let graph = ClearPathGraph::corner_graph(&map, 0);
        println!("Built graph");
        for p in map.unvisited().iter().chain(map.all_spaces().iter()) {
            assert!(graph.edges_from(&p).count() > 0);
        }
        assert!(graph.is_connected());
    }
}
