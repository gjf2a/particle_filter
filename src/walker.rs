// Walker's algorithm for selection
// A. J. Walker, “An efficient method for generating discrete random variables with general distributions,” ACM Transactions on Mathematical Software, vol. 3, no. 3, pp. 253–256, 1977.
// https://crates.io/crates/weighted_rand
//
// Adapted for rank-proportionate selection. Given n ranked values:
// * Find the sum of the ranks
// * Assign a probability of 1/sum for the lowest ranked, 2/sum for the 
//   next-lowest, and so forth.

use std::{cmp::Ordering, collections::{HashMap, VecDeque}, iter::repeat_n};

use hash_histogram::HashHistogram;
use rand::{RngExt, rng};

#[derive(Clone, Debug)]
pub struct WalkerAlias {
    n: usize,
    prob: Vec<f64>,
    alias: Vec<usize>,
}

impl WalkerAlias {
    pub fn rank_proportionate(n: usize) -> Self {
        let sum_n = Self::gauss_sum_n(n);
        let mut prob = vec![];
        let mut alias = vec![];
        for i in 0..n {
            let c = i + 1;
            let p = (c * n) as f64 / sum_n as f64;
            if p >= 1.0 {
                prob.push(1.0);
                alias.push(i);
            } else {
                prob.push(p);
                alias.push(n - c);
            }
        }
        Self {n, prob, alias}
    }

    // Imagine weights of 4, 5, 2, and 9. 
    // Sum = 20
    // Probabilities: 0.2, 0.25, 0.1, 0.45
    // alias[0] = 3
    // alias[1] = 1
    // alias[2] = 3
    // alias[3] = 3
    pub fn weighted(weights: &HashHistogram<usize, f64>) -> Self {
        let total = weights.total_count();
        let expected = 1.0 / weights.len() as f64;
        let mut probs_falling = weights.iter().map(|(i, w)| (*i, *w / total)).collect::<Vec<_>>();
        probs_falling.sort_by(|(_, w1), (_,w2)| w2.partial_cmp(w1).unwrap_or(Ordering::Equal));
        let mut prob = repeat_n(0.0, weights.len()).collect::<Vec<_>>();
        let mut alias = (0..weights.len()).collect::<Vec<_>>();
        let mut overages = vec![];
        for (i, p) in probs_falling.iter() {
            println!("i: {i} expected: {expected} p: {p:.2}, {overages:?}");
            if *p > expected {
                prob[*i] = 1.0;
                overages.push((*i, *p - expected));
            } else if *p < expected {
                let (over_i, over_w) = overages.last_mut().unwrap();
                *over_w -= expected - *p;
                prob[*i] = *p / expected;
                alias[*i] = *over_i;
                if *over_w <= 0.0 {
                    overages.pop();
                }
            }
            println!("i: {i} alias: {} prob: {}", alias[*i], prob[*i]);
        }
        Self {n: weights.len(), prob, alias}
    }

    fn gauss_sum_n(n: usize) -> usize {
        n * (n + 1) / 2
    }

    pub fn n(&self) -> usize {
        self.n
    }

    pub fn sum_n(&self) -> usize {
        Self::gauss_sum_n(self.n)
    }

    pub fn choose(&self) -> usize {
        let mut rng = rng();
        let choice = rng.random_range(0..self.n);
        if rng.random::<f64>() < self.prob[choice] {
            choice 
        } else {
            self.alias[choice]
        }
    }
}

#[cfg(test)]
mod tests {
    use hash_histogram::HashHistogram;

    use crate::walker::WalkerAlias;

    #[test]
    fn test_rank_proportionate() {
        for n in 2..=8 {
            let mut histogram: HashHistogram<usize, usize> = HashHistogram::new();
            let distro = WalkerAlias::rank_proportionate(n);
            let num_samples = 100000;
            for _ in 0..num_samples {
                histogram.bump(&distro.choose());
            }
            let denominator = distro.sum_n();
            let tolerance = num_samples / (denominator * 5);
            println!("n: {n} ({})", distro.sum_n());
            for i in 0..distro.n() {
                let numerator = i + 1;
                let target = num_samples * numerator / denominator;
                let lo = target - tolerance;
                let hi = target + tolerance;
                println!("{numerator}/{denominator}\t{lo}\t{target}\t{hi}");
                assert!(lo <= histogram.count(&i) && histogram.count(&i) <= hi);
            }
        }
    }

    #[test]
    fn test_weighted() {
        let weights = [(0, 0.2), (1, 0.25), (2, 0.1), (3, 0.45)].iter().copied().collect::<HashHistogram<_,_>>();
        let distro = WalkerAlias::weighted(&weights);
        let mut histogram: HashHistogram<usize, usize> = HashHistogram::new();
        let num_samples = 100000;
        for _ in 0..num_samples {
            histogram.bump(&distro.choose());
        }
        let denominator = distro.sum_n();
        let tolerance = num_samples / (denominator * 5);
        for i in 0..distro.n() {
            let target = (num_samples as f64 * weights.count(&i) / weights.total_count()) as usize;
            let lo = target - tolerance;
            let hi = target + tolerance;
            println!("{lo}\t{target}\t{hi}");
            println!("count: {}", histogram.count(&i));
            assert!(lo <= histogram.count(&i) && histogram.count(&i) <= hi);
        }
    }
}