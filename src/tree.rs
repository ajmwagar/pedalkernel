//! WDF tree structures for connecting elements

use crate::WdfNode;

/// Series adaptor for WDF trees
#[derive(Debug)]
pub struct SeriesAdaptor<L: WdfNode, R: WdfNode> {
    left: L,
    right: R,
    port_resistance: f64,
}

impl<L: WdfNode, R: WdfNode> SeriesAdaptor<L, R> {
    pub fn new(left: L, right: R) -> Self {
        Self {
            left,
            right,
            port_resistance: 0.0,
        }
    }
}

impl<L: WdfNode, R: WdfNode> WdfNode for SeriesAdaptor<L, R> {
    fn reflection(&self, incident_wave: f64) -> f64 {
        // Series connection logic
        todo!("Implement series reflection")
    }
    
    fn update(&mut self, sample_rate: f64) {
        self.left.update(sample_rate);
        self.right.update(sample_rate);
        // Rp = Rl + Rr
    }
    
    fn reset(&mut self) {
        self.left.reset();
        self.right.reset();
    }
}

/// Parallel adaptor for WDF trees
#[derive(Debug)]
pub struct ParallelAdaptor<L: WdfNode, R: WdfNode> {
    left: L,
    right: R,
    port_resistance: f64,
}

impl<L: WdfNode, R: WdfNode> ParallelAdaptor<L, R> {
    pub fn new(left: L, right: R) -> Self {
        Self {
            left,
            right,
            port_resistance: 0.0,
        }
    }
}

impl<L: WdfNode, R: WdfNode> WdfNode for ParallelAdaptor<L, R> {
    fn reflection(&self, incident_wave: f64) -> f64 {
        // Parallel connection logic
        todo!("Implement parallel reflection")
    }
    
    fn update(&mut self, sample_rate: f64) {
        self.left.update(sample_rate);
        self.right.update(sample_rate);
        // 1/Rp = 1/Rl + 1/Rr
    }
    
    fn reset(&mut self) {
        self.left.reset();
        self.right.reset();
    }
}

/// WDF tree root - top level of the circuit
pub struct WdfTree<T: WdfNode> {
    root: T,
    sample_rate: f64,
}

impl<T: WdfNode> WdfTree<T> {
    pub fn new(root: T, sample_rate: f64) -> Self {
        let mut tree = Self { root, sample_rate };
        tree.update();
        tree
    }
    
    pub fn process(&mut self, input: f64) -> f64 {
        self.root.reflection(input)
    }
    
    pub fn update(&mut self) {
        self.root.update(self.sample_rate);
    }
    
    pub fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        self.update();
    }
    
    pub fn reset(&mut self) {
        self.root.reset();
    }
}
