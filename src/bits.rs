use std::{
    cmp::max,
    ops::{BitAnd, BitOr, BitXor},
    str::FromStr,
    u64,
};

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Hash, Eq, Ord, PartialOrd, Clone, Debug, Default)]
pub struct BitArray {
    bits: Vec<u64>,
}

impl PartialEq for BitArray {
    fn eq(&self, other: &Self) -> bool {
        for (w1, w2) in self.bits.iter().zip(other.bits.iter()) {
            if w1 != w2 {
                return false;
            }
        }
        if self.bits.len() >= other.bits.len() {
            (other.bits.len()..self.bits.len()).all(|i| self.bits[i] == 0)
        } else {
            (self.bits.len()..other.bits.len()).all(|i| other.bits[i] == 0)
        }
    }
}

impl BitArray {
    pub fn zeros(num_zeros: usize) -> Self {
        let mut num_words = num_zeros / Self::bits_per_word();
        if num_zeros % Self::bits_per_word() > 0 {
            num_words += 1;
        }
        Self {
            bits: std::iter::repeat_n(0, num_words).collect(),
        }
    }

    pub fn ones(num_ones: usize) -> Self {
        let num_words = num_ones / Self::bits_per_word();
        let mut bits = std::iter::repeat_n(u64::MAX, num_words).collect::<Vec<_>>();
        let leftover = num_ones % Self::bits_per_word();
        if leftover > 0 {
            bits.push(2_u64.pow(leftover as u32) - 1);
        }
        BitArray { bits }
    }

    pub fn len(&self) -> usize {
        self.bits
            .iter()
            .map(|word| word.count_ones() as usize)
            .sum()
    }

    pub fn iter<'a>(&'a self) -> OnesIterator<'a> {
        OnesIterator::new(self)
    }

    fn make_mask(index: &usize) -> u64 {
        1 << BitArray::find_offset(index)
    }

    fn find_offset(index: &usize) -> usize {
        index % BitArray::bits_per_word()
    }

    fn find_word(index: &usize) -> usize {
        index / BitArray::bits_per_word()
    }

    pub fn bits_per_word() -> usize {
        std::mem::size_of::<u64>() * 8
    }

    pub fn insert(&mut self, value: usize) {
        while self.bits.len() < 1 + BitArray::find_word(&value) {
            self.bits.push(0);
        }
        let mask = BitArray::make_mask(&value);
        self.bits[BitArray::find_word(&value)] |= mask;
    }

    pub fn remove(&mut self, value: &usize) {
        let mask = BitArray::make_mask(value);
        let word = BitArray::find_word(value);
        if word < self.bits.len() {
            self.bits[word] &= !mask;
        }
    }

    pub fn contains(&self, value: &usize) -> bool {
        let word = BitArray::find_word(value);
        word < self.bits.len() && self.bits[word] & BitArray::make_mask(value) > 0
    }
}

pub struct OnesIterator<'a> {
    src: &'a BitArray,
    word_index: usize,
    word_value: u64,
}

impl<'a> OnesIterator<'a> {
    pub fn new(src: &'a BitArray) -> Self {
        Self {
            src,
            word_index: 0,
            word_value: if src.bits.len() > 0 { src.bits[0] } else { 0 },
        }
    }
}

impl<'a> Iterator for OnesIterator<'a> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.word_value != 0 {
                break;
            } else {
                self.word_index += 1;
                if self.word_index >= self.src.bits.len() {
                    return None;
                } else {
                    self.word_value = self.src.bits[self.word_index];
                }
            }
        }
        let offset = self.word_value.trailing_zeros();
        self.word_value &= self.word_value - 1;
        Some(self.word_index * 64 + offset as usize)
    }
}

impl FromIterator<usize> for BitArray {
    fn from_iter<T: IntoIterator<Item = usize>>(iter: T) -> Self {
        let mut result = BitArray::default();
        for value in iter {
            result.insert(value);
        }
        result
    }
}

impl<'a> FromIterator<&'a usize> for BitArray {
    fn from_iter<T: IntoIterator<Item = &'a usize>>(iter: T) -> Self {
        iter.into_iter().copied().collect()
    }
}

impl BitXor for &BitArray {
    type Output = BitArray;

    fn bitxor(self, rhs: Self) -> Self::Output {
        create_from(self, rhs, |a, b| a ^ b)
    }
}

impl BitAnd for &BitArray {
    type Output = BitArray;

    fn bitand(self, rhs: Self) -> Self::Output {
        create_from(self, rhs, |a, b| a & b)
    }
}

impl BitOr for &BitArray {
    type Output = BitArray;

    fn bitor(self, rhs: Self) -> Self::Output {
        create_from(self, rhs, |a, b| a | b)
    }
}

fn create_from(one: &BitArray, two: &BitArray, op: fn(u64, u64) -> u64) -> BitArray {
    let mut result = BitArray::default();
    let result_bits_len = max(one.bits.len(), two.bits.len());
    for i in 0..result_bits_len {
        if i >= one.bits.len() {
            result.bits.push(op(two.bits[i], 0));
        } else if i >= two.bits.len() {
            result.bits.push(op(one.bits[i], 0));
        } else {
            result.bits.push(op(one.bits[i], two.bits[i]));
        }
    }
    result
}

impl FromStr for BitArray {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut result = BitArray::default();
        for (i, ch) in s.chars().rev().enumerate() {
            match ch {
                '1' => {
                    result.insert(i);
                }
                '0' => {}
                other => return Err(anyhow::anyhow!("Illegal digit parsing BitArray: {other}")),
            }
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn test_by_3() {
        let nums = (0..20000).filter(|i| i % 3 == 0).collect::<BTreeSet<_>>();
        let num_bits = nums.iter().collect::<BitArray>();
        for num in 0..20000 {
            assert_eq!(nums.contains(&num), num_bits.contains(&num));
        }
        assert_eq!(nums.len(), num_bits.len());
        assert!(!num_bits.contains(&100000));
    }

    fn generator<F: Fn(&usize) -> bool>(n: usize, condition: F) -> BitArray {
        (0..n).filter(|i| condition(i)).collect()
    }

    #[test]
    fn test_big_ops() {
        let test_size = 200_000;
        let threes = generator(test_size, |i| i % 3 == 0);
        let twos = generator(test_size, |i| i % 2 == 0);
        let either = generator(test_size, |i| i % 3 == 0 || i % 2 == 0);
        let both = generator(test_size, |i| i % 2 == 0 && i % 3 == 0);
        let one_or_other = generator(test_size, |i| (i % 2 == 0) != (i % 3 == 0));
        assert_eq!(&twos & &threes, both);
        assert_eq!(&twos | &threes, either);
        assert_eq!(&twos ^ &threes, one_or_other);
    }

    #[test]
    fn test_expand_set() {
        let mut b = BitArray::default();
        b.insert(0);
        b.insert(2);
        b.insert(3);
        assert_eq!(b.len(), 3);
        assert_eq!(b, "1101".parse().unwrap());

        b.remove(&0);
        b.remove(&3);
        assert_eq!(b.len(), 1);
        assert_eq!(b, "0100".parse().unwrap());
    }

    #[test]
    fn test_parse() {
        for (nums, s) in [
            (vec![0, 3, 4, 6], "1011001"),
            (vec![1, 2, 4, 5, 6], "1110110"),
        ] {
            let b = nums.iter().collect::<BitArray>();
            assert_eq!(b, s.parse::<BitArray>().unwrap());
            assert_eq!(b.len(), nums.len());
            for (n1, n2) in b.iter().zip(nums.iter()) {
                assert_eq!(n1, *n2);
            }
        }
    }

    #[test]
    fn test_ones() {
        let num_ones = 65;
        let bits = BitArray::ones(num_ones);
        assert!((0..num_ones).all(|i| bits.contains(&i)));
    }

    #[test]
    fn test_empty_ones() {
        let bits = BitArray::default();
        let ones = bits.iter().collect::<Vec<_>>();
        assert_eq!(ones.len(), 0);
    }

    #[test]
    fn test_remove_eq() {
        let nums = [0, 10, 65, 129];
        let mut bits = nums.iter().collect::<BitArray>();
        assert_eq!(nums.len(), bits.len());
        for n in nums.iter() {
            assert!(bits.contains(n));
        }
        let other_bits = nums[..3].iter().collect::<BitArray>();
        assert!(bits != other_bits);
        bits.remove(&nums[3]);
        assert_eq!(bits, other_bits);
    }
}
