use pyo3::prelude::*;
use rand::{thread_rng, Rng};
use rand::distributions::Alphanumeric;

#[pyfunction]
pub fn generate_nonce() -> String {
    let rng = thread_rng();
    let nonce: String = rng
        .sample_iter(&Alphanumeric)
        .take(25)
        .map(char::from)
        .collect();
    nonce
}
