//! NumPy `.npy` file I/O.
//!
//! This module provides functions to read and write NumPy `.npy` files,
//! enabling interoperability with Python-based analysis tools.
//!
//! # Example
//!
//! ```rust,ignore
//! use pedalkernel_validate::npy;
//!
//! // Write test output
//! let output = vec![0.1, 0.2, 0.3, 0.4, 0.5];
//! npy::write_f64("output.npy", &output).unwrap();
//!
//! // Read reference data
//! let reference = npy::read_f64("golden.npy").unwrap();
//!
//! // Check if a file exists
//! if npy::exists("reference.npy") {
//!     // Load and compare
//! }
//! ```
//!
//! # Python Interoperability
//!
//! Files written by this module can be loaded in Python:
//!
//! ```python
//! import numpy as np
//! data = np.load("output.npy")
//! ```
//!
//! And Python-generated files can be read:
//!
//! ```python
//! import numpy as np
//! golden = np.sin(np.linspace(0, 2*np.pi, 1000))
//! np.save("golden.npy", golden)
//! ```

use ndarray::Array1;
use ndarray_npy::{ReadNpyError, WriteNpyError};
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum NpyError {
    #[error("Failed to read NPY file: {0}")]
    ReadError(#[from] ReadNpyError),
    #[error("Failed to write NPY file: {0}")]
    WriteError(#[from] WriteNpyError),
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Read a 1D float64 array from an NPY file.
pub fn read_f64(path: impl AsRef<Path>) -> Result<Vec<f64>, NpyError> {
    let arr: Array1<f64> = ndarray_npy::read_npy(path)?;
    Ok(arr.to_vec())
}

/// Write a 1D float64 array to an NPY file.
pub fn write_f64(path: impl AsRef<Path>, data: &[f64]) -> Result<(), NpyError> {
    let path = path.as_ref();

    // Ensure parent directory exists
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let arr = Array1::from_vec(data.to_vec());
    ndarray_npy::write_npy(path, &arr)?;
    Ok(())
}

/// Check if an NPY file exists.
pub fn exists(path: impl AsRef<Path>) -> bool {
    path.as_ref().exists()
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn roundtrip_f64_array() {
        let data: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();

        let tmpfile = NamedTempFile::new().unwrap();
        let path = tmpfile.path().to_path_buf();

        // Write
        write_f64(&path, &data).unwrap();

        // Read back
        let loaded = read_f64(&path).unwrap();

        assert_eq!(data.len(), loaded.len());
        for (a, b) in data.iter().zip(loaded.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }
}
