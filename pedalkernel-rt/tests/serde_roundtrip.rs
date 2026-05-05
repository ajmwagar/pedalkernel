//! Serialization round-trip test for the `serde` feature.
//!
//! Verifies that DynNode trees can be serialized to postcard (a compact
//! no_std-friendly format) and deserialized back, producing identical audio
//! output from the reconstructed tree.

#![cfg(feature = "serde")]

extern crate alloc;

use pedalkernel_rt::dyn_node::DynNode;

/// Build a simple Series(VS, Series(R, C)) WDF tree — the classic RC lowpass.
fn build_rc_lowpass() -> DynNode {
    // RC lowpass: VS in series with (R in series with C)
    // R = 1kOhm, C = 100nF, fs = 48kHz
    // rp for capacitor = 1/(2*C*fs) = 1/(2 * 100e-9 * 48000) ~= 104.17
    let sample_rate = 48000.0;
    let capacitance = 100e-9;
    let rp_c = 1.0 / (2.0 * capacitance * sample_rate);

    let r = DynNode::Resistor(Some(alloc::string::String::from("R1")), 1000.0);
    let c = DynNode::Capacitor(Some(alloc::string::String::from("C1")), capacitance, rp_c);
    let vs = DynNode::VoltageSource(0.0, 1.0); // 1 ohm source impedance

    let rc = DynNode::Series(Box::new(r), Box::new(c));
    let mut tree = DynNode::Series(Box::new(vs), Box::new(rc));
    tree.recompute();
    tree
}

/// Process N samples through a DynNode tree (short-circuit root), returning output.
fn process_samples(tree: &mut DynNode, input: &[f64]) -> alloc::vec::Vec<f64> {
    let mut output = alloc::vec::Vec::with_capacity(input.len());
    for &sample in input {
        // Set voltage source input
        tree.set_voltage(sample);
        // Wave-up pass: compute reflected wave from leaves to root
        let a_root = tree.reflected();
        // Short-circuit root: b = -a
        let b_root = -a_root;
        // Wave-down pass: propagate incident wave from root to leaves
        tree.set_incident(b_root);
        // Output = voltage at root junction
        output.push(tree.short_circuit_junction_voltage(a_root).unwrap_or(0.0));
    }
    output
}

#[test]
fn dyn_node_postcard_roundtrip() {
    let original = build_rc_lowpass();

    // Serialize to postcard (compact, no_std-friendly binary format)
    let bytes = postcard::to_allocvec(&original).expect("serialize failed");
    assert!(
        bytes.len() < 2048,
        "serialized size unexpectedly large: {} bytes",
        bytes.len()
    );

    // Deserialize
    let mut deserialized: DynNode =
        postcard::from_bytes(&bytes).expect("deserialize failed");
    deserialized.recompute();

    // Build a second copy of the original for comparison
    let mut original_copy = build_rc_lowpass();

    // Generate a short impulse response (32 samples)
    let mut input = alloc::vec![0.0f64; 32];
    input[0] = 1.0; // impulse

    let out_original = process_samples(&mut original_copy, &input);
    let out_deserialized = process_samples(&mut deserialized, &input);

    // Verify outputs match exactly
    assert_eq!(
        out_original.len(),
        out_deserialized.len(),
        "output lengths differ"
    );

    for (i, (a, b)) in out_original.iter().zip(&out_deserialized).enumerate() {
        assert!(
            (a - b).abs() < 1e-12,
            "sample {i}: original={a}, deserialized={b}, diff={}",
            (a - b).abs()
        );
    }

    // Sanity: the impulse response should not be all zeros
    let energy: f64 = out_original.iter().map(|x| x * x).sum();
    assert!(energy > 1e-6, "impulse response is silent (energy={energy})");
}
