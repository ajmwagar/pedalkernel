//! Property-based stress tests: random passive RC ladders must remain stable,
//! finite, and energy-bounded under random bounded excitation.

mod audio_analysis;

use audio_analysis::*;
use rand::{rngs::StdRng, Rng, SeedableRng};

#[test]
fn random_passive_ladders_are_bibo_stable() {
    let mut rng = StdRng::seed_from_u64(0x9e37_79b9_7f4a_7c15);
    for case in 0..32 {
        let stages = rng.gen_range(1..=4);
        let pedal_src = random_rc_ladder(&mut rng, stages);
        let input = random_bounded_input(&mut rng, 4096);
        let output = compile_and_process(&pedal_src, &input, SAMPLE_RATE, &[]);

        assert!(
            output.iter().all(|x| x.is_finite()),
            "Case {case}: output contains NaN/inf"
        );

        let e_in = energy(&input);
        let e_out = energy(&output);
        assert!(e_out.is_finite(), "Case {case}: energy NaN");
        assert!(
            e_out <= e_in * 20.0 + 1e-9,
            "Case {case}: passive ladder blew up ({e_in} -> {e_out})"
        );

        let out_peak = peak(&output);
        assert!(
            out_peak < 3.0,
            "Case {case}: passive ladder produced large peak {out_peak}"
        );
    }
}

fn random_rc_ladder(rng: &mut StdRng, stages: usize) -> String {
    let mut components = String::new();
    let mut nets = String::new();

    for i in 0..stages {
        let r_val = rng.gen_range(100.0..50_000.0);
        let c_val = rng.gen_range(1e-10..1e-6);
        components.push_str(&format!("    R{i}: resistor({r_val:.6})\n"));
        components.push_str(&format!("    C{i}: cap({c_val:.12})\n"));

        let in_node = if i == 0 {
            "in".to_string()
        } else {
            format!("N{}", i)
        };
        let out_node = if i == stages - 1 {
            "out".to_string()
        } else {
            format!("N{}", i + 1)
        };

        nets.push_str(&format!("    {in_node} -> R{i}.a\n"));
        nets.push_str(&format!("    R{i}.b -> C{i}.a, {out_node}\n"));
        nets.push_str(&format!("    C{i}.b -> gnd\n"));
    }

    format!(
        "pedal \"Random RC\" {{\n  components {{\n{components}  }}\n  nets {{\n{nets}  }}\n}}\n"
    )
}

fn random_bounded_input(rng: &mut StdRng, samples: usize) -> Vec<f64> {
    (0..samples).map(|_| rng.gen_range(-0.8..0.8)).collect()
}
