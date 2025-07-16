use pyo3::prelude::*;
use rand::{thread_rng, Rng};
use rand::distributions::Alphanumeric;

/// Generate a cryptographic nonce for API requests
pub fn generate_nonce() -> String {
    thread_rng()
        .sample_iter(&Alphanumeric)
        .take(25)
        .map(char::from)
        .collect()
}

/// Python wrapper for generate_nonce
#[pyfunction]
pub fn generate_nonce_py() -> String {
    generate_nonce()
}

/// Sanitize a name for use in file paths
#[allow(dead_code)]
pub fn sanitize_name(name: &str) -> String {
    name.chars()
        .map(|c| match c {
            '/' | '\\' | ':' | '*' | '?' | '"' | '<' | '>' | '|' => '_',
            _ => c,
        })
        .collect()
}
