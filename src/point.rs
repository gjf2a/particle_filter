use std::{
    fmt::Display,
    ops::{
        Add, AddAssign, BitOr, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
    },
    str::FromStr,
};

use itertools::Itertools;
use serde::{Deserialize, Serialize};

use crate::bit_grid::{NumType, RowMajorCoordIter, span};

pub type GridPoint = Point<i64, 2>;
pub type FloatPoint = Point<f64, 2>;

#[derive(Serialize, Deserialize, Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Point<N: NumType, const S: usize> {
    #[serde(with = "serde_arrays")]
    coords: [N; S],
}

impl<N: NumType, const S: usize> Display for Point<N, S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let joined = self.coords.map(|n| format!("{n}")).join(",");
        write!(f, "({joined})")
    }
}

impl<N: NumType, const S: usize> FromStr for Point<N, S> {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        let parts = s[1..s.len() - 1].split(',').collect::<Vec<_>>();
        if parts.len() == S {
            parts
                .iter()
                .map(|s| {
                    Ok(s.trim()
                        .parse::<N>()
                        .map_err(|_| anyhow::anyhow!("Parse error when parsing '{s}'"))?)
                })
                .collect()
        } else {
            Err(anyhow::anyhow!(
                "Expecting {S} values, but received {} values instead from {}",
                parts.len(),
                s
            ))
        }
    }
}

impl<N: NumType, const S: usize> FromIterator<N> for Point<N, S> {
    fn from_iter<T: IntoIterator<Item = N>>(iter: T) -> Self {
        let mut result = Self::default();
        for (i, n) in (0..S).zip(iter) {
            result[i] = n;
        }
        result
    }
}

impl<N: NumType, const S: usize> Point<N, S> {
    pub fn new(coords: [N; S]) -> Self {
        Self { coords }
    }

    pub fn euclidean_distance(&self, other: Point<N, S>) -> f64 {
        (0..S)
            .map(|i| ((self[i] - other[i]).to_f64().expect("Shouldn't happen")).powf(2.0))
            .sum::<f64>()
            .sqrt()
    }

    pub fn manhattan_distance(&self, other: Point<N, S>) -> N {
        (0..S).map(|i| abs_difference(self[i], other[i])).sum()
    }

    pub fn iter(&self) -> impl Iterator<Item = N> {
        self.coords.iter().copied()
    }

    pub fn dot(&self, other: &Point<N, S>) -> N {
        self.coords
            .iter()
            .zip(other.iter())
            .map(|(x, y)| *x * y)
            .sum()
    }

    pub fn element_max(&self, other: &Point<N, S>) -> Point<N, S> {
        self.iter()
            .zip(other.iter())
            .map(|(a, b)| if a < b { b } else { a })
            .collect()
    }

    pub fn element_min(&self, other: &Point<N, S>) -> Point<N, S> {
        self.iter()
            .zip(other.iter())
            .map(|(a, b)| if a < b { a } else { b })
            .collect()
    }
}

impl<N: NumType, const S: usize> From<N> for Point<N, S> {
    fn from(value: N) -> Self {
        Self { coords: [value; S] }
    }
}

macro_rules! impl_point_iter {
    ($numtype:tt) => {
        impl Point<$numtype, 2> {
            pub fn point_iter(&self, end: &Self) -> impl Iterator<Item = Point<$numtype, 2>> {
                (self[1]..=end[1]).flat_map(|y| (self[0]..=end[0]).map(move |x| Point::new([x, y])))
            }
        }
    };
}

impl_point_iter!(i64);
impl_point_iter!(u64);

const OFFSETS: [i64; 3] = [-1, 0, 1];

pub fn manhattan_offsets<const S: usize>() -> impl Iterator<Item = Point<i64, S>> {
    OFFSETS
        .iter()
        .copied()
        .permutations(S)
        .filter(|c| c.iter().filter(|n| **n != 0).count() == 1)
        .map(|v| {
            let mut values = [0; S];
            for i in 0..S {
                values[i] = v[i];
            }
            Point::<i64, S>::new(values)
        })
}

impl<const S: usize> Point<i64, S> {
    pub fn manhattan_neighbors(&self) -> impl Iterator<Item = Point<i64, S>> + use<'_, S> {
        manhattan_offsets().map(|offset| *self + offset)
    }
}

pub fn abs_difference<N: NumType>(a: N, b: N) -> N {
    if a < b { b - a } else { a - b }
}

impl<N: NumType, const S: usize> Index<usize> for Point<N, S> {
    type Output = N;

    fn index(&self, index: usize) -> &Self::Output {
        &self.coords[index]
    }
}

impl<N: NumType, const S: usize> IndexMut<usize> for Point<N, S> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.coords[index]
    }
}

impl<N: NumType, const S: usize> Default for Point<N, S> {
    fn default() -> Self {
        Self {
            coords: [N::default(); S],
        }
    }
}

impl<N: NumType, const S: usize> Add for Point<N, S> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut result = self;
        result += rhs;
        result
    }
}

impl<N: NumType, const S: usize> AddAssign for Point<N, S> {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..S {
            self[i] += rhs[i];
        }
    }
}

macro_rules! create_sub_signed {
    ($numtype:tt) => {
        impl<const S: usize> Neg for Point<$numtype, S> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                let mut result = Self::default();
                for i in 0..S {
                    result[i] = -self[i];
                }
                result
            }
        }

        impl<const S: usize> Sub for Point<$numtype, S> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                self + -rhs
            }
        }

        impl<const S: usize> SubAssign for Point<$numtype, S> {
            fn sub_assign(&mut self, rhs: Self) {
                *self += -rhs;
            }
        }
    };
}

create_sub_signed!(isize);
create_sub_signed!(i64);
create_sub_signed!(i32);
create_sub_signed!(i16);
create_sub_signed!(i8);

create_sub_signed!(f64);
create_sub_signed!(f32);

// Guaranteed to avoid underflow
macro_rules! create_sub_unsigned {
    ($numtype:tt) => {
        impl<const S: usize> SubAssign for Point<$numtype, S> {
            fn sub_assign(&mut self, rhs: Self) {
                for i in 0..S {
                    if self[i] > rhs[i] {
                        self[i] -= rhs[i];
                    } else {
                        self[i] = 0;
                    }
                }
            }
        }

        impl<const S: usize> Sub for Point<$numtype, S> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                let mut result = self;
                result -= rhs;
                result
            }
        }
    };
}

create_sub_unsigned!(usize);
create_sub_unsigned!(u64);
create_sub_unsigned!(u32);
create_sub_unsigned!(u16);
create_sub_unsigned!(u8);

impl<N: NumType, const S: usize> Mul<N> for Point<N, S> {
    type Output = Self;

    fn mul(self, rhs: N) -> Self::Output {
        let mut result = self;
        result *= rhs;
        result
    }
}

impl<N: NumType, const S: usize> MulAssign<N> for Point<N, S> {
    fn mul_assign(&mut self, rhs: N) {
        for i in 0..S {
            self[i] *= rhs;
        }
    }
}

impl<N: NumType, const S: usize> DivAssign<N> for Point<N, S> {
    fn div_assign(&mut self, rhs: N) {
        for i in 0..S {
            self[i] /= rhs;
        }
    }
}

impl<N: NumType, const S: usize> Div<N> for Point<N, S> {
    type Output = Self;

    fn div(self, rhs: N) -> Self::Output {
        let mut result = self;
        result /= rhs;
        result
    }
}

#[derive(Serialize, Deserialize, Default, Clone, Copy, Debug, Eq, PartialEq)]
#[serde(bound(serialize = "N: NumType", deserialize = "N: NumType"))]
pub struct BoundingBox<N: NumType> {
    min: Point<N, 2>,
    max: Point<N, 2>,
}

impl<N: NumType> BoundingBox<N> {
    pub fn new(min: Point<N, 2>, max: Point<N, 2>) -> Self {
        Self { min, max }
    }

    pub fn observe(&mut self, p: &Point<N, 2>) {
        self.min = self.min.element_min(&p);
        self.max = self.max.element_max(&p);
    }

    pub fn min(&self) -> Point<N, 2> {
        self.min
    }

    pub fn max(&self) -> Point<N, 2> {
        self.max
    }

    pub fn singular(&self) -> bool {
        self.min == self.max
    }

    pub fn center(&self) -> Point<N, 2> {
        (self.min + self.max) / (N::one() + N::one())
    }

    pub fn in_bounds(&self, p: &Point<N, 2>) -> bool {
        self.min[0] <= p[0] && p[0] <= self.max[0] && self.min[1] <= p[1] && p[1] <= self.max[1]
    }

    pub fn grow(&mut self, growth: N) {
        assert!(growth > N::zero());
        self.min[0] -= growth;
        self.min[1] -= growth;
        self.max[0] += growth;
        self.max[1] += growth;
    }
}

impl BoundingBox<i64> {
    pub fn width(&self) -> i64 {
        span(self.min[0], self.max[0])
    }

    pub fn height(&self) -> i64 {
        span(self.min[1], self.max[1])
    }

    pub fn area(&self) -> i64 {
        self.width() * self.height()
    }

    pub fn coord_iter(&self) -> RowMajorCoordIter {
        RowMajorCoordIter::new(self.min()[0], self.min()[1], self.width(), self.height())
    }
}

impl BoundingBox<f64> {
    pub fn width(&self) -> f64 {
        self.max[0] - self.min[0]
    }

    pub fn height(&self) -> f64 {
        self.max[1] - self.min[1]
    }

    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }
}

impl<N: NumType> Add<Point<N, 2>> for BoundingBox<N> {
    type Output = Self;

    fn add(self, rhs: Point<N, 2>) -> Self::Output {
        Self {
            min: self.min + rhs,
            max: self.max + rhs,
        }
    }
}

impl<N: NumType> BitOr for BoundingBox<N> {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self {
            min: self.min.element_min(&rhs.min),
            max: self.max.element_max(&rhs.max),
        }
    }
}

impl<N: NumType> FromIterator<Point<N, 2>> for Option<BoundingBox<N>> {
    fn from_iter<T: IntoIterator<Item = Point<N, 2>>>(iter: T) -> Self {
        let mut result: Option<BoundingBox<N>> = None;
        for point in iter {
            if let Some(result) = &mut result {
                result.observe(&point);
            } else {
                result = Some(BoundingBox::new(point, point));
            }
        }
        result
    }
}

impl<'a, N: NumType> FromIterator<&'a Point<N, 2>> for Option<BoundingBox<N>> {
    fn from_iter<T: IntoIterator<Item = &'a Point<N, 2>>>(iter: T) -> Self {
        iter.into_iter().copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::{
        bit_grid::ColumnMajorCoordIter,
        point::{BoundingBox, Point}, pt,
    };

    use super::GridPoint;

    #[test]
    fn test_neighbor() {
        test_neighbor_help(GridPoint::new([3, 2]), &[(2, 2), (4, 2), (3, 1), (3, 3)]);
        test_neighbor_help(GridPoint::new([0, 0]), &[(-1, 0), (1, 0), (0, -1), (0, 1)]);
    }

    fn test_neighbor_help(gp: GridPoint, expected: &[(i64, i64)]) {
        let neighbors = gp.manhattan_neighbors().collect::<HashSet<_>>();
        for (x, y) in expected.iter() {
            let np = GridPoint::new([*x, *y]);
            assert!(neighbors.contains(&np));
        }
        assert_eq!(neighbors.len(), expected.len());
    }

    #[test]
    fn test_parse() {
        for (x, y) in [(1, 2), (3, 4), (5, 6)] {
            let text = format!("({x}, {y})");
            let gp = text.parse::<GridPoint>().unwrap();
            assert_eq!(gp[0], x);
            assert_eq!(gp[1], y);
        }
    }

    #[test]
    fn test_from() {
        for n in [0, 1, 2, 3] {
            let point: Point<i64, 2> = n.into();
            point.iter().for_each(|v| assert_eq!(v, n));
        }
    }

    #[test]
    fn test_point_range() {
        let start: Point<i64, 2> = Point::new([-1, -2]);
        let end = Point::new([1, 2]);
        let expected = [
            (-1, -2),
            (0, -2),
            (1, -2),
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (0, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
            (-1, 2),
            (0, 2),
            (1, 2),
        ];
        assert!(
            expected
                .iter()
                .zip(start.point_iter(&end))
                .all(|((ex, ey), p)| *ex == p[0] && *ey == p[1])
        );
    }

    #[test]
    fn test_element_min_max() {
        let a = GridPoint::new([2, 7]);
        let b = GridPoint::new([4, 3]);
        assert_eq!(GridPoint::new([2, 3]), a.element_min(&b));
        assert_eq!(GridPoint::new([4, 7]), a.element_max(&b));
    }

    #[test]
    fn test_singular() {
        let bb: BoundingBox<i64> = BoundingBox::default();
        assert!(bb.singular());
    }

    #[test]
    fn test_bounding_box() {
        let bb: Option<BoundingBox<i64>> =
            [pt!(1, 2), pt!(-3, -2), pt!(-1, -1), pt!(-5, -1), pt!(0, 4)]
                .iter()
                .copied()
                .collect();
        let bb = bb.unwrap();
        assert_eq!(bb.min()[0], -5);
        assert_eq!(bb.max()[0], 1);
        assert_eq!(bb.min()[1], -2);
        assert_eq!(bb.max()[1], 4);
        assert_eq!(bb.center(), pt!(-2, 1));

        let moved = bb + pt!(-1, 1);
        assert_eq!(moved.min()[0], -6);
        assert_eq!(moved.max()[0], 0);
        assert_eq!(moved.min()[1], -1);
        assert_eq!(moved.max()[1], 5);
        assert_eq!(moved.center(), pt!(-3, 2));

        for (p, inside) in [
            (pt!(0, 0), true),
            (pt!(1, -1), false),
            (pt!(2, 2), false),
            (pt!(-6, -1), true),
            (pt!(-6, 5), true),
            (pt!(0, -1), true),
            (pt!(0, 5), true),
            (pt!(-7, -1), false),
            (pt!(0, 6), false),
        ]
        .iter()
        {
            assert_eq!(moved.in_bounds(p), *inside);
        }
    }

    #[test]
    fn test_bounding_box_observe() {
        let mut bb = BoundingBox {
            min: Point { coords: [-2, -1] },
            max: Point { coords: [-2, -1] },
        };
        bb.observe(&pt!(-1, -1));
        let expected = BoundingBox {
            min: Point { coords: [-2, -1] },
            max: Point { coords: [-1, -1] },
        };
        assert_eq!(expected, bb);
    }

    #[test]
    fn test_bounding_box_collect() {
        let bb: Option<BoundingBox<i64>> = [pt!(-2, -1)].iter().copied().collect();
        let bb = bb.unwrap();
        let expected = BoundingBox {
            min: Point { coords: [-2, -1] },
            max: Point { coords: [-2, -1] },
        };
        assert_eq!(bb, expected);
    }

    #[test]
    fn test_ordering() {
        let cds = ColumnMajorCoordIter {
            max_x: 3,
            min_y: 1,
            max_y: 3,
            x: 1,
            y: 1,
        }
        .collect::<Vec<_>>();
        let mut pts = cds.clone();
        pts.sort();
        assert_eq!(cds, pts);
    }
}
