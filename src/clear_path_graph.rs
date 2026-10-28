use std::collections::{HashMap, VecDeque};

use crate::{BitGridMap, bit_grid::BitGrid, point::GridPoint};

pub struct ClearPathGraph {
    distances: HashMap<GridPoint,HashMap<GridPoint,f64>>,
    frontier: BitGrid,
    visitable: BitGrid,
}

fn set_from_to<V>(map: &mut HashMap<GridPoint,HashMap<GridPoint,V>>, from: &GridPoint, to: &GridPoint, value: V) {
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
    pub fn new(map: &BitGridMap) -> Self {
        let frontier = map.unvisited();
        let visitable = &frontier | &map.all_spaces();
        let visitable_vec = visitable.iter().collect::<Vec<_>>();
        let mut distances = HashMap::new();
        for i in 0..visitable_vec.len() {
            for j in (i + 1)..visitable_vec.len() {
                if map.clear_path_between(&visitable_vec[i], &visitable_vec[j]) {
                    let distance = visitable_vec[i].euclidean_distance(visitable_vec[j]);
                    set_from_to(&mut distances, &visitable_vec[i], &visitable_vec[j], distance);
                    set_from_to(&mut distances, &visitable_vec[j], &visitable_vec[i], distance);
                }
            }
        }
        Self {distances, frontier, visitable}
    }

    pub fn edges_from(&self, p: &GridPoint) -> impl Iterator<Item=(&GridPoint,&f64)> {
        // From https://users.rust-lang.org/t/how-to-return-an-empty-iterator-if-my-vector-isnt-found/100248
        self.distances.get(p).into_iter().flatten()
    }

    pub fn frontier(&self) -> &BitGrid {
        &self.frontier
    }

    pub fn visitable(&self) -> &BitGrid {
        &self.visitable
    }

    pub fn is_connected(&self) -> bool {
        match self.visitable.iter().next() {
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
                visited == self.visitable
            }
        }
    }
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
        for n1 in graph.visitable().iter() {
            set_from_to(&mut distances, &n1, &n1, 0.0);
            set_from_to(&mut predecessors, &n1, &n1, n1);
            for (n2, cost) in graph.edges_from(&n1) {
                set_from_to(&mut distances, &n1, n2, *cost);
                set_from_to(&mut predecessors, &n1, n2, n1);
            }
        }
        for k in graph.visitable().iter() {
            for i in graph.visitable().iter() {
                for j in graph.visitable().iter() {
                    if let Some(i2k) = distances.get(&i).unwrap().get(&k) {
                        if let Some(k2j) = distances.get(&k).unwrap().get(&j) {
                            let alt_i2j = i2k + k2j;
                            if distances.get(&i).unwrap().get(&j).map_or(true, |i2j| *i2j > alt_i2j) {
                                set_from_to(&mut distances, &i, &j, alt_i2j);
                                let k_pred = predecessors.get(&k).unwrap().get(&j).copied().unwrap();
                                set_from_to(&mut predecessors, &i, &j, k_pred);
                            }
                        }
                    }
                }
            }
        }
        Self {distances, predecessors}
    }

    pub fn distance_between(&self, p1: &GridPoint, p2: &GridPoint) -> f64 {
        self.distances.get(p1).unwrap().get(p2).copied().unwrap()
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
        let graph = ClearPathGraph::new(&map);
        println!("Built graph");
        for p in map.unvisited().iter().chain(map.all_spaces().iter()) {
            assert!(graph.edges_from(&p).count() > 0);
        }
        assert!(graph.is_connected());
    }
}