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
    m.add_class::<client::OnshapeClient>()?;
    m.add_class::<model::DocumentMetaData>()?;
    m.add_class::<model::DefaultWorkspace>()?;
    m.add_class::<model::Element>()?;
    m.add_class::<model::Variable>()?;
    m.add_class::<model::Assembly>()?;
    m.add_class::<model::RootAssembly>()?;
    m.add_class::<model::Document>()?;
    m.add_class::<model::MassProperties>()?;
    m.add_class::<model::TranslationJob>()?;
    Ok(())
}
