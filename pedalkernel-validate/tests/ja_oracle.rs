use std::process::Command;

#[test]
fn ja_python_oracle_is_available_when_enabled() {
    if std::env::var("PEDALKERNEL_RUN_PY_ORACLES").as_deref() != Ok("1") {
        eprintln!("set PEDALKERNEL_RUN_PY_ORACLES=1 to run the Python/Numpy J-A oracle");
        return;
    }

    let manifest = env!("CARGO_MANIFEST_DIR");
    let oracle = format!("{manifest}/oracles/ja_validation_oracle.py");
    let output = Command::new("python3")
        .arg(oracle)
        .output()
        .expect("failed to launch python3 for J-A oracle");

    assert!(
        output.status.success(),
        "J-A oracle failed\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        String::from_utf8_lossy(&output.stdout).contains("ALL CHECKS PASSED"),
        "J-A oracle did not report success\nstdout:\n{}",
        String::from_utf8_lossy(&output.stdout)
    );
}
