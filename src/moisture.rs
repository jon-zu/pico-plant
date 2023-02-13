use crate::sum_tree::SumTree;

const fn unwrap(opt: Option<usize>) -> usize {
    match opt {
        Some(x) => x,
        None => {
            core::mem::forget(opt);
            panic!("Trying to unwrap a None")
        }
    }
}

const LEAFS: usize = 8;
const N: usize = 2 * unwrap(LEAFS.checked_next_power_of_two());

pub struct MoistureSensor {
    min: u16,
    max: u16,
    avg: SumTree<u32, N>,
    i: usize,
}

impl Default for MoistureSensor {
    fn default() -> Self {
        Self::new(1087 - 50, 3296)
    }
}

impl MoistureSensor {
    pub fn new(min: u16, max: u16) -> Self {
        let init = (max - min) / 2;
        Self {
            min,
            max,
            avg: SumTree::new(init as u32),
            i: 0,
        }
    }

    fn range(&self) -> u16 {
        self.max - self.min
    }

    fn map(&self, sample: u16) -> u16 {
        sample.saturating_sub(self.min).min(self.range())
    }

    fn percent(&self, v: u16) -> u16 {
        let perc = v / (self.range() / 100);
        // Invert
        100 - perc
    }

    pub fn avg(&self) -> u16 {
        (self.avg.get_root_sum() / LEAFS as u32) as u16
    }

    pub fn avg_percent(&self) -> u16 {
        self.percent(self.avg() as u16)
    }

    pub fn add_sample(&mut self, sample: u16) {
        let sample = self.map(sample);
        self.avg.update_leaf_node_sample(self.i, sample as u32);
        self.i += 1;
        self.i %= LEAFS;
    }
}
