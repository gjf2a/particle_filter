use crate::{bits::BitArray, pt};
use num_traits::{Num, cast::ToPrimitive};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use std::{
    fmt::Display,
    iter::Sum,
    ops::{AddAssign, BitAnd, BitOr, BitXor, DivAssign, MulAssign, SubAssign},
    str::FromStr,
};
use trait_set::trait_set;

trait_set! {
    pub trait NumType = Serialize + DeserializeOwned + FromStr + Display + ToPrimitive + Default + Num + Copy + AddAssign + SubAssign + MulAssign + DivAssign + PartialOrd + Sum;
}

use crate::point::{BoundingBox, GridPoint};

pub fn span(a: i64, b: i64) -> u64 {
    let max = if a > b { a } else { b };
    let min = if a < b { a } else { b };
    (1 + max - min) as u64
}

#[derive(Clone, Eq, Debug, Serialize, Deserialize)]
pub struct BitGrid {
    bits: BitArray,
    bounds: BoundingBox<i64>,
}

impl PartialEq for BitGrid {
    fn eq(&self, other: &Self) -> bool {
        if self.bounds == other.bounds {
            self.bits == other.bits
        } else {
            let self_count = self.iter().count();
            let other_count = other.iter().count();
            if self_count == other_count {
                self.iter().zip(other.iter()).all(|(a, b)| a == b)
            } else {
                false
            }
        }
    }
}

impl Default for BitGrid {
    fn default() -> Self {
        Self {
            bounds: BoundingBox::new(pt!(0, 0), pt!(0, 0)),
            bits: BitArray::default(),
        }
    }
}

impl BitGrid {
    fn zero_grid(bounds: BoundingBox<i64>) -> Self {
        let num_zeros =
            span(bounds.min()[0], bounds.max()[0]) * span(bounds.min()[1], bounds.max()[1]);
        Self {
            bounds,
            bits: BitArray::zeros(num_zeros as usize),
        }
    }

    pub fn one_grid(bounds: BoundingBox<i64>) -> Self {
        let num_ones =
            span(bounds.min()[0], bounds.max()[0]) * span(bounds.min()[1], bounds.max()[1]);
        Self {
            bounds,
            bits: BitArray::ones(num_ones as usize),
        }
    }

    pub fn translated(&self, translation: GridPoint) -> Self {
        Self {
            bits: self.bits.clone(),
            bounds: self.bounds + translation,
        }
    }

    pub fn scaled(&self, scale: f64) -> Self {
        let mut result = Self {
            bits: BitArray::default(),
            bounds: self.bounds * scale,
        };
        for p in result.bounds.row_major_coord_iter() {
            if self.contains(&(p * scale)) {
                result.insert(p);
            }
        }
        result
    }

    pub fn dilated(&self) -> Self {
        let mut result = Self::default();
        for one in self.iter() {
            for n in one.manhattan_neighbors() {
                result.insert(n);
            }
        }
        result
    }

    pub fn center(&self) -> GridPoint {
        self.bounds.center()
    }

    pub fn x_axis_reflection(&self) -> Self {
        self.iter()
            .map(|p| pt!(p[0], self.bounds.max()[1] - p[1]))
            .collect()
    }

    pub fn y_axis_reflection(&self) -> Self {
        self.iter()
            .map(|p| pt!(self.bounds.max()[0] - p[0], p[1]))
            .collect()
    }

    pub fn num_bits(&self) -> u64 {
        self.width() as u64 * self.height() as u64
    }

    pub fn bounding_box(&self) -> BoundingBox<i64> {
        self.bounds.clone()
    }

    pub fn manhattan_neighbors(&self, p: &GridPoint) -> impl Iterator<Item = (GridPoint, bool)> {
        manhattan_iter(p).map(|p| (p, self.contains(&p)))
    }

    pub fn all_neighbors(&self, p: &GridPoint) -> impl Iterator<Item = (GridPoint, bool)> {
        all_iter(p).map(|p| (p, self.contains(&p)))
    }

    fn bits(&self) -> &BitArray {
        &self.bits
    }

    pub fn contains(&self, p: &GridPoint) -> bool {
        if self.bounds.in_bounds(p) {
            self.bits.contains(&self.index_1d(p))
        } else {
            false
        }
    }

    pub fn insert(&mut self, p: GridPoint) {
        if self.bounds.in_bounds(&p) {
            self.bits.insert(self.index_1d(&p));
        } else {
            if self.iter().next().is_none() {
                self.bounds = BoundingBox::new(p, p);
                self.insert_unchecked(&p);
            } else {
                let new_bounds: Option<BoundingBox<i64>> = self.iter().collect();
                let mut new_bounds = new_bounds.unwrap();
                new_bounds.observe(&p);
                let mut new_self = Self::zero_grid(new_bounds);
                for one in self.iter() {
                    assert!(new_self.bounds.in_bounds(&one));
                    new_self.insert_unchecked(&one);
                }
                std::mem::swap(&mut new_self, self);
                self.insert_unchecked(&p);
            }
        }
    }

    fn insert_unchecked(&mut self, p: &GridPoint) {
        self.bits.insert(self.index_1d(&p));
    }

    pub fn remove(&mut self, p: &GridPoint) {
        if self.contains(p) {
            self.bits.remove(&self.index_1d(&p));
        }
    }

    pub fn width(&self) -> u64 {
        self.bounds.width()
    }

    pub fn height(&self) -> u64 {
        self.bounds.height()
    }

    pub fn centroid(&self) -> GridPoint {
        self.iter().sum::<GridPoint>() / (self.len() as i64)
    }

    pub fn col_major_coord_iter(&self) -> ColumnMajorCoordIter {
        self.bounds.col_major_coord_iter()
    }

    pub fn row_major_coord_iter(&self) -> RowMajorCoordIter {
        self.bounds.row_major_coord_iter()
    }

    pub fn words_used(&self) -> usize {
        let base = self.bits().len() / 64;
        let extra = if self.bits().len() % 64 > 0 { 1 } else { 0 };
        base + extra
    }

    pub fn coord_contains_iter(&self) -> impl Iterator<Item = (GridPoint, bool)> {
        self.row_major_coord_iter().map(|p| (p, self.contains(&p)))
    }

    pub fn iter(&self) -> impl Iterator<Item = GridPoint> {
        self.bits.iter().map(|i| self.index_2d(i))
    }

    pub fn ones_touching_zeros(&self) -> impl Iterator<Item = GridPoint> {
        self.iter().filter(|p| {
            self.manhattan_neighbors(p)
                .filter(|(_, value)| *value)
                .count()
                < 4
        })
    }

    pub fn len(&self) -> usize {
        self.bits().len()
    }

    fn stringify(&self) -> String {
        let mut s = format!("{}\n", self.bounds.min());
        for (p, value) in self.coord_contains_iter() {
            if p[1] > self.bounds.min()[1] && p[0] == self.bounds.min()[0] {
                s.push('\n');
            }
            let c = if value { '1' } else { '0' };
            s.push(c);
        }
        s
    }

    pub fn overlapping_counts(&self, other: &Self) -> usize {
        (self & other).len()
    }

    fn index_1d(&self, p: &GridPoint) -> usize {
        let grid_x = p[0] - self.bounds.min()[0];
        let grid_y = p[1] - self.bounds.min()[1];
        (grid_y * self.width() as i64 + grid_x) as usize
    }

    fn index_2d(&self, i: usize) -> GridPoint {
        let i = i as i64;
        let uy = i / self.width() as i64;
        let ux = i % self.width() as i64;
        pt!(ux + self.bounds.min()[0], uy + self.bounds.min()[1])
    }

    fn to_bit(c: char) -> anyhow::Result<bool> {
        Ok(match c {
            '1' | 'X' | '*' => true,
            '0' | 'O' | '.' => false,
            _ => return Err(anyhow::anyhow!("Illegal char: {c}")),
        })
    }
}

impl BitAnd for &BitGrid {
    type Output = BitGrid;

    fn bitand(self, rhs: Self) -> Self::Output {
        if self.bounds == rhs.bounds {
            BitGrid {
                bits: &self.bits & &rhs.bits,
                bounds: self.bounds,
            }
        } else {
            self.iter().filter(|p| rhs.contains(p)).collect()
        }
    }
}

impl BitXor for &BitGrid {
    type Output = BitGrid;

    fn bitxor(self, rhs: Self) -> Self::Output {
        if self.bounds == rhs.bounds {
            BitGrid {
                bits: &self.bits ^ &rhs.bits,
                bounds: self.bounds,
            }
        } else {
            let mut union = self
                .iter()
                .filter(|p| !rhs.contains(p))
                .collect::<BitGrid>();
            for p in rhs.iter().filter(|p| !self.contains(p)) {
                union.insert(p);
            }
            union
        }
    }
}

impl BitOr for &BitGrid {
    type Output = BitGrid;

    fn bitor(self, rhs: Self) -> Self::Output {
        if self.bounds == rhs.bounds {
            BitGrid {
                bits: &self.bits | &rhs.bits,
                bounds: self.bounds,
            }
        } else {
            let mut union = self.iter().collect::<BitGrid>();
            for one in rhs.iter() {
                union.insert(one);
            }
            union
        }
    }
}

impl Display for BitGrid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.stringify())
    }
}

impl FromStr for BitGrid {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut lines = s.trim().lines().filter(|line| line.len() > 0);
        let start_point = lines
            .by_ref()
            .next()
            .ok_or(anyhow::anyhow!("No content"))?
            .parse::<GridPoint>()?;
        let height = lines.clone().count();
        let width = lines
            .clone()
            .next()
            .ok_or(anyhow::anyhow!("No grid entries"))?
            .trim()
            .len();
        let mut result = Self::zero_grid(BoundingBox::new(
            start_point,
            start_point + pt!((width - 1) as i64, (height - 1) as i64),
        ));
        for (y, line) in lines.enumerate() {
            for (x, c) in line.trim().char_indices() {
                if Self::to_bit(c)? {
                    result.insert_unchecked(&(start_point + pt!(x as i64, y as i64)));
                }
            }
        }
        Ok(result)
    }
}

impl FromIterator<(GridPoint, bool)> for BitGrid {
    fn from_iter<T: IntoIterator<Item = (GridPoint, bool)>>(iter: T) -> Self {
        iter.into_iter()
            .filter(|(_, value)| *value)
            .map(|(p, _)| p)
            .collect()
    }
}

impl FromIterator<GridPoint> for BitGrid {
    fn from_iter<T: IntoIterator<Item = GridPoint>>(iter: T) -> Self {
        let mut result = BitGrid::default();
        for p in iter {
            result.insert(p);
        }
        result
    }
}

impl<'a> FromIterator<&'a GridPoint> for BitGrid {
    fn from_iter<T: IntoIterator<Item = &'a GridPoint>>(iter: T) -> Self {
        iter.into_iter().copied().collect()
    }
}

#[macro_export]
macro_rules! make_coord_iter {
    ($name:tt, $major_max:tt, $minor_min:tt, $minor_max:tt, $minor:tt, $major:tt) => {
        pub struct $name {
            pub $major_max: i64,
            pub $minor_min: i64,
            pub $minor_max: i64,
            pub $minor: i64,
            pub $major: i64,
        }

        impl Iterator for $name {
            type Item = GridPoint;

            fn next(&mut self) -> Option<Self::Item> {
                if self.$major > self.$major_max {
                    None
                } else {
                    let result = Some(pt!(self.x, self.y));
                    self.$minor += 1;
                    if self.$minor > self.$minor_max {
                        self.$minor = self.$minor_min;
                        self.$major += 1;
                    }
                    result
                }
            }
        }
    };
}

make_coord_iter!(ColumnMajorCoordIter, max_x, min_y, max_y, y, x);

impl ColumnMajorCoordIter {
    pub fn new(x_start: i64, y_start: i64, width: u64, height: u64) -> Self {
        Self {
            max_x: x_start + width as i64 - 1,
            max_y: y_start + height as i64 - 1,
            min_y: y_start,
            x: x_start,
            y: y_start,
        }
    }
}

make_coord_iter!(RowMajorCoordIter, max_y, min_x, max_x, x, y);
impl RowMajorCoordIter {
    pub fn new(x_start: i64, y_start: i64, width: u64, height: u64) -> Self {
        Self {
            max_x: x_start + width as i64 - 1,
            max_y: y_start + height as i64 - 1,
            min_x: x_start,
            x: x_start,
            y: y_start,
        }
    }
}

const MANHATTAN_OFFSETS: [(i64, i64); 4] = [(-1, 0), (0, -1), (1, 0), (0, 1)];
const ALL_OFFSETS: [(i64, i64); 8] = [
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
];

fn manhattan_iter(p: &GridPoint) -> impl Iterator<Item = GridPoint> {
    offset_iter(p, &MANHATTAN_OFFSETS)
}

fn all_iter(p: &GridPoint) -> impl Iterator<Item = GridPoint> {
    offset_iter(p, &ALL_OFFSETS)
}

fn offset_iter(p: &GridPoint, offsets: &[(i64, i64)]) -> impl Iterator<Item = GridPoint> {
    offsets
        .iter()
        .copied()
        .map(move |(off_x, off_y)| pt!(off_x + p[0], off_y + p[1]))
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeSet, HashMap};

    use super::*;

    #[test]
    fn test_from_str() {
        let grid_str = "(0,0)\n1101\n1011\n0010\n";
        let grid = grid_str.parse::<BitGrid>().unwrap();
        assert_eq!(format!("{grid}\n"), grid_str);
        assert_eq!(grid.height(), 3);
        assert_eq!(grid.width(), 4);
        assert_eq!(grid.len(), 7);
        for (x, y, value) in [
            (0, 0, true),
            (1, 0, true),
            (2, 0, false),
            (3, 0, true),
            (0, 1, true),
            (1, 1, false),
            (2, 1, true),
            (3, 1, true),
            (0, 2, false),
            (1, 2, false),
            (2, 2, true),
            (3, 2, false),
            (4, 1, false),
        ] {
            assert_eq!(grid.contains(&pt!(x, y)), value);
        }

        // out of bounds - still false
        for (x, y) in [(-1, 0), (3, 3), (1, 3), (1, -3)] {
            assert_eq!(grid.contains(&pt!(x, y)), false);
            assert!(!grid.bounds.in_bounds(&pt!(x, y)));
        }
    }

    #[test]
    fn test_from_str_2() {
        let str = "(-2,-1)\n11\n01\n10\n00";
        let src: BitGrid = str.parse().unwrap();
        assert_eq!(format!("{src}"), str);
        assert_eq!(src.bounds, BoundingBox::new(pt!(-2, -1), pt!(-1, 2)));
        let cleared: BitGrid = src.iter().collect();
        for (s, c) in src.iter().zip(cleared.iter()) {
            assert_eq!(s, c);
        }
        assert_eq!(src.iter().count(), cleared.iter().count());
        assert_eq!(cleared.bounds, BoundingBox::new(pt!(-2, -1), pt!(-1, 1)));
        assert_eq!(src, cleared);
    }

    #[test]
    fn test_no_ones() {
        let grid = BitGrid::default();
        let ones = grid.iter().collect::<Vec<_>>();
        assert_eq!(ones.len(), 0);
    }

    #[test]
    fn test_from_iter() {
        let pvs = [(pt!(2, 2), true), (pt!(-1, 3), false), (pt!(3, -2), true)];
        let test_points = pvs.iter().copied().collect::<HashMap<_, _>>();
        let grid = pvs.iter().copied().collect::<BitGrid>();
        for p in grid.iter() {
            assert!(test_points.get(&p).unwrap_or(&false));
        }
        assert_eq!(grid.len(), 2);
        assert_eq!(grid.len(), grid.iter().count());
        assert_eq!(grid.width(), 2);
        assert_eq!(grid.height(), 5);

        let ones1 = grid
            .coord_contains_iter()
            .filter(|(_, v)| *v)
            .map(|(p, _)| p)
            .collect::<Vec<_>>();
        let ones2 = grid.iter().collect::<Vec<_>>();
        for one in ones2.iter() {
            assert!(grid.contains(one));
        }
        assert_eq!(ones1, ones2);
        for (p, value) in grid.coord_contains_iter() {
            match test_points.get(&p) {
                Some(expected) => {
                    assert_eq!(value, *expected);
                }
                None => {
                    assert_eq!(value, false);
                }
            }
        }
    }

    #[test]
    fn test_from_coord_iter() {
        let coords = vec![pt!(-2, -1), pt!(1, -1), pt!(-1, 1), pt!(1, 2), pt!(3, 4)];
        let coord_set = coords.iter().collect::<BitGrid>();
        let rebuilt = coord_set.iter().collect::<Vec<_>>();
        assert_eq!(coords, rebuilt);
    }

    #[test]
    fn test_overlapping_counts() {
        for (one, two, count) in [
            ("(0,0)\n101\n010", "(0,0)\n110\n110", 2),
            ("(0,0)\n101\n010", "(0,0)\n010\n101", 0),
            ("(0,0)\n101\n010", "(0,0)\n101\n010", 3),
            ("(0,0)\n111\n111", "(0,0)\n111\n111", 6),
            ("(0,0)\n000\n000", "(0,0)\n000\n000", 0),
        ] {
            let one = one.parse::<BitGrid>().unwrap();
            let two = two.parse::<BitGrid>().unwrap();
            assert_eq!(count, one.overlapping_counts(&two));
        }
    }

    #[test]
    fn test_manhattan_iter() {
        let test_grid = "(0,0)\n010\n100\n101".parse::<BitGrid>().unwrap();
        for (p, neighbors) in [
            (
                pt!(0, 0),
                vec![
                    (pt!(-1, 0), false),
                    (pt!(0, -1), false),
                    (pt!(1, 0), true),
                    (pt!(0, 1), true),
                ],
            ),
            (
                pt!(1, 1),
                vec![
                    (pt!(0, 1), true),
                    (pt!(1, 0), true),
                    (pt!(2, 1), false),
                    (pt!(1, 2), false),
                ],
            ),
            (
                pt!(1, 2),
                vec![
                    (pt!(0, 2), true),
                    (pt!(1, 1), false),
                    (pt!(2, 2), true),
                    (pt!(1, 3), false),
                ],
            ),
        ] {
            let actual = test_grid.manhattan_neighbors(&p).collect::<Vec<_>>();
            assert_eq!(actual, neighbors);
        }
    }

    #[test]
    fn test_ones_touching_zeros() {
        let test_grid = "(0,0)
        10100000100000011
        11111010101011011
        10110000001000011
        "
        .parse::<BitGrid>()
        .unwrap();
        let found = test_grid.ones_touching_zeros().collect::<BTreeSet<_>>();
        let expected = [
            (0, 0),
            (2, 0),
            (8, 0),
            (15, 0),
            (16, 0),
            (0, 1),
            (1, 1),
            (3, 1),
            (4, 1),
            (6, 1),
            (8, 1),
            (10, 1),
            (12, 1),
            (13, 1),
            (15, 1),
            (16, 1),
            (0, 2),
            (2, 2),
            (3, 2),
            (10, 2),
            (15, 2),
            (16, 2),
        ]
        .iter()
        .map(|(x, y)| pt!(*x, *y))
        .collect::<BTreeSet<_>>();
        assert_eq!(expected, found);
        let one_count = test_grid.len();
        assert_eq!(found.len() + 1, one_count);
        assert_eq!(23, test_grid.bits().len());
        assert_eq!(1, test_grid.words_used());
    }

    #[test]
    fn test_bit_or_even() {
        let a: BitGrid = "(0,0)\n101\n011\n000".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = "(0,0)\n101\n111\n010".parse().unwrap();
        assert_eq!((&a | &b), c);
    }

    #[test]
    fn test_bit_and_even() {
        let a: BitGrid = "(0,0)\n101\n011\n000".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = [pt!(2, 0), pt!(2, 1)].iter().collect();
        assert_eq!((&a & &b), c);
    }

    #[test]
    fn test_bit_xor_even() {
        let a: BitGrid = "(0,0)\n101\n011\n000".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = "(0,0)\n10\n11\n01".parse().unwrap();
        assert_eq!((&a ^ &b), c);
    }

    #[test]
    fn test_bit_or_uneven() {
        let a: BitGrid = "(0,0)\n101\n011".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = "(0,0)\n101\n111\n010".parse().unwrap();
        assert_eq!((&a | &b), c);
    }

    #[test]
    fn test_bit_and_uneven() {
        let a: BitGrid = "(0,0)\n101\n011".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = [pt!(2, 0), pt!(2, 1)].iter().collect();
        assert_eq!((&a & &b), c);
    }

    #[test]
    fn test_bit_xor_uneven() {
        let a: BitGrid = "(0,0)\n101\n011".parse().unwrap();
        let b: BitGrid = "(0,0)\n001\n101\n010".parse().unwrap();
        let c: BitGrid = "(0,0)\n10\n11\n01".parse().unwrap();
        assert_eq!((&a ^ &b), c);
    }

    #[test]
    fn test_resize() {
        let mut a: BitGrid = "(0,0)\n101\n011\n000".parse().unwrap();
        assert_eq!(4, a.bits().len());
        assert_eq!(1, a.words_used());
        a.insert(pt!(-2, -2));
        assert_eq!(5, a.bits.len());
        assert_eq!(1, a.words_used());
        let ex1 = "(-2,-2)\n10000\n00000\n00101\n00011";
        assert_eq!(ex1, format!("{a}").as_str());
        assert_eq!(a.bounds.min(), pt!(-2, -2));
        assert_eq!(a.bounds.max(), pt!(2, 1));
        a.insert(pt!(-2, 3));

        let ex2 = "(-2,-2)\n10000\n00000\n00101\n00011\n00000\n10000";
        assert_eq!(ex2, format!("{a}").as_str());
        assert_eq!(a.bounds.min(), pt!(-2, -2));
        assert_eq!(a.bounds.max(), pt!(2, 3));
        assert_eq!(6, a.bits.len());
        assert_eq!(1, a.words_used());
    }

    #[test]
    fn test_translated() {
        let a: BitGrid = "(0,0)\n010\n111\n010".parse().unwrap();
        let zeroed = a.translated(pt!(-1, -1));
        let expected_ones = vec![pt!(0, -1), pt!(-1, 0), pt!(0, 0), pt!(1, 0), pt!(0, 1)];
        let zeroed_ones = zeroed.iter().collect::<Vec<_>>();
        assert_eq!(expected_ones, zeroed_ones);
    }

    #[test]
    fn test_reflection() {
        let map: BitGrid = "(0,0)\n0010\n1101\n0101\n0110".parse().unwrap();
        let expect_x: BitGrid = "(0,0)\n0110\n0101\n1101\n0010".parse().unwrap();
        let expect_y: BitGrid = "(0,0)\n0100\n1011\n1010\n0110".parse().unwrap();

        assert_eq!(map.x_axis_reflection(), expect_x);
        assert_eq!(map.y_axis_reflection(), expect_y);
    }

    #[test]
    fn test_row_major_iteration() {
        let points: Vec<GridPoint> = RowMajorCoordIter {
            min_x: 0,
            max_x: 3,
            max_y: 2,
            y: 0,
            x: 0,
        }
        .collect();
        let grid: BitGrid = points.iter().collect();
        let grid_points: Vec<GridPoint> = grid.iter().collect();
        assert_eq!(points, grid_points);
    }

    #[test]
    fn test_no_origin() {
        let tests = [vec![pt!(2, 3), pt!(4, 5)], vec![pt!(-2, 3), pt!(-1, -4)]];
        for test in tests {
            let bits: BitGrid = test.iter().copied().collect();
            for bit in bits.iter() {
                assert!(test.contains(&bit));
            }
            for bit in test.iter() {
                assert!(bits.contains(bit));
            }
        }
    }

    #[test]
    fn test_eq() {
        for (a, b, expected) in [
            ("(0, 0)\n11\n01", "(0, 0)\n11\n01", true),
            ("(0, 0)\n11\n01", "(1, 0)\n11\n01", false),
            ("(0, 0)\n11\n01", "(0, 0)\n110\n010", true),
            ("(0, 0)\n11\n01", "(-1, 0)\n0110\n0010", true),
            ("(0, 0)\n11\n01", "(-1, -1)\n0110\n0010", false),
        ] {
            let a: BitGrid = a.parse().unwrap();
            let b: BitGrid = b.parse().unwrap();
            assert_eq!(a == b, expected);
        }
    }
}
