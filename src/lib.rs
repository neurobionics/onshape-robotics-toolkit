use pyo3::prelude::*;
use rand::{thread_rng, Rng};
use rand::distributions::Alphanumeric;

mod client;

#[pyfunction]
fn generate_nonce() -> String {
    let mut rng = thread_rng();
    let nonce: String = rng
        .sample_iter(&Alphanumeric)
        .take(25)
        .map(char::from)
        .collect();
    nonce
}

#[pymodule]
fn native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(generate_nonce, m)?)?;
    m.add_class::<client::OnshapeClient>()?;
    Ok(())
}
