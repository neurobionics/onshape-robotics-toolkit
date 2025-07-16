use pyo3::prelude::*;

mod client;
mod endpoint;
mod endpoints;
mod error;
mod model;
mod params;
mod utils;

pub use client::{AsyncOnshapeClient, OnshapeClient};
pub use endpoint::*;
pub use endpoints::*;
pub use error::{ApiError, OnshapeError};
pub use model::*;
pub use params::*;

// Re-export derive_builder for users who want to create their own endpoints
pub use derive_builder;

#[pymodule]
fn native(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(utils::generate_nonce_py, m)?)?;
    m.add_class::<client::OnshapeClient>()?;
    Ok(())
}

// Re-export core traits for library users
pub use endpoint::{Endpoint, Client, AsyncClient};
