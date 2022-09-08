// Based on: https://stackoverflow.com/questions/6737283/weighted-randomness-in-java

use std::collections::BTreeMap;
use ordered_float::OrderedFloat;
use rand::Rng;

pub struct Distribution<T> {
    distro: BTreeMap<OrderedFloat<f64>, T>,
    total_weight: f64
}

impl <T:Clone> Distribution<T> {
    pub fn new() -> Self {
        Distribution {distro: BTreeMap::new(), total_weight: 0.0}
    }

    pub fn add(&mut self, value: &T, weight: f64) {
        assert!(weight > 0.0);
        self.distro.insert(OrderedFloat(self.total_weight), value.clone());
        self.total_weight += weight;
    }

    pub fn random_pick(&self) -> T {
        let mut rng = rand::thread_rng();
        let key_picked = closest_key_below(&self.distro, rng.gen_range(0.0..self.total_weight));
        self.distro.get(&key_picked.unwrap()).unwrap().clone()
    }
}

fn closest_key_below<T>(tree: &BTreeMap<OrderedFloat<f64>, T>, target: f64) -> Option<OrderedFloat<f64>> {
    tree.range(..=OrderedFloat(target)).rev().next().map(|(k,_)| *k)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use ordered_float::OrderedFloat;
    use crate::distribution::closest_key_below;

    #[test]
    fn test_closest_key_below() {
        let t: BTreeMap<OrderedFloat<f64>, String> = [(1.0, "a"), (0.5, "b"), (3.5, "c"), (4.8, "d")]
            .iter()
            .map(|(k, v)| (OrderedFloat(*k), v.to_string()))
            .collect();
        assert_eq!(closest_key_below(&t, 0.6).unwrap(), 0.5);
        assert_eq!(closest_key_below(&t, 1.0).unwrap(), 1.0);
        assert_eq!(closest_key_below(&t, 1.00001).unwrap(), 1.0);
        assert_eq!(closest_key_below(&t, 10.0).unwrap(), 4.8);
        assert_eq!(closest_key_below(&t, 4.1).unwrap(), 3.5);
        assert_eq!(closest_key_below(&t, 3.4).unwrap(), 1.0);
    }
}