use pyo3::prelude::*;
use rand::{thread_rng, Rng};
use rand::distributions::Alphanumeric;

mod client;
mod utils;

#[pymodule]
fn native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(utils::generate_nonce, m)?)?;
    m.add_class::<client::OnshapeClient>()?;
    Ok(())
}
