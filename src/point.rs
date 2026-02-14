use std::{
    fmt::Display,
    iter::Sum,
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    str::FromStr,
};

use itertools::Itertools;
use num_traits::{Num, cast::ToPrimitive};
use trait_set::trait_set;

trait_set! {
    pub trait NumType = FromStr + Display + ToPrimitive + Default + Num + Copy + AddAssign + SubAssign + MulAssign + DivAssign + PartialOrd + Sum;
}

pub type GridPoint = Point<u64, 2>;
pub type FloatPoint = Point<f64, 2>;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Point<N: NumType, const S: usize> {
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
                    Ok(s.trim().parse::<N>()
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
}

impl<N: NumType, const S: usize> From<N> for Point<N, S> {
    fn from(value: N) -> Self {
        Self { coords: [value; S] }
    }
}

const OFFSETS: [i64; 3] = [-1, 0, 1];

impl<const S: usize> Point<u64, S> {
    pub fn manhattan_neighbors(&self) -> impl Iterator<Item = Point<u64, S>> + use<'_, S> {
        OFFSETS
            .iter()
            .permutations(S)
            .filter(|c| c.iter().filter(|n| ***n != 0).count() == 1)
            .map(|c| {
                let mut values = [0; S];
                for i in 0..S {
                    values[i] = self[i] as i64 + c[i];
                }
                values
            })
            .filter(|c| (0..S).all(|i| c[i] >= 0))
            .map(|c| Point::<u64, S>::new(c.map(|v| v as u64)))
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

impl<N: NumType + Neg<Output = N>, const S: usize> Neg for Point<N, S> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..S {
            result[i] = -self[i];
        }
        result
    }
}

impl<N: NumType + Neg<Output = N>, const S: usize> Sub for Point<N, S> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        self + -rhs
    }
}

impl<N: NumType + Neg<Output = N>, const S: usize> SubAssign for Point<N, S> {
    fn sub_assign(&mut self, rhs: Self) {
        *self += -rhs;
    }
}

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

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::point::Point;

    use super::GridPoint;

    #[test]
    fn test_neighbor() {
        test_neighbor_help(GridPoint::new([3, 2]), &[(2, 2), (4, 2), (3, 1), (3, 3)]);
        test_neighbor_help(GridPoint::new([0, 0]), &[(1, 0), (0, 1)]);
    }

    fn test_neighbor_help(gp: GridPoint, expected: &[(u64, u64)]) {
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
}
