use pyo3::prelude::*;

mod client;
mod endpoint;
mod endpoints;
mod error;
mod model;
mod params;
mod utils;
mod parse;

pub use client::{AsyncOnshapeClient, OnshapeClient};
pub use endpoint::*;
pub use endpoints::*;
pub use error::{ApiError, OnshapeError};
pub use model::{DocumentMetaData, DefaultWorkspace, Element, Variable, Document, MassProperties, TranslationJob, NameOrId};
// Keep the original Assembly and RootAssembly from model for backward compatibility
pub use model::{Assembly as ModelAssembly, RootAssembly as ModelRootAssembly};
pub use params::*;
// Re-export parse module with specific items to avoid conflicts
pub mod parse_module {
    pub use crate::parse::*;
}

// Re-export derive_builder for users who want to create their own endpoints
pub use derive_builder;

#[pymodule]
fn native(_py: Python, m: &PyModule) -> PyResult<()> {
    // Add the function to the module
    m.add_function(wrap_pyfunction!(parse_module::high_level_api::get_instances_rust, m)?)?;
    m.add_function(wrap_pyfunction!(parse_module::high_level_api::get_mates_and_relations_rust, m)?)?;
    m.add_function(wrap_pyfunction!(parse_module::high_level_api::get_performance_metrics, m)?)?;

    // Add classes to the module
    m.add_class::<OnshapeClient>()?;
    m.add_class::<DocumentMetaData>()?;
    m.add_class::<DefaultWorkspace>()?;
    m.add_class::<Element>()?;
    m.add_class::<Variable>()?;
    m.add_class::<Document>()?;
    m.add_class::<MassProperties>()?;
    m.add_class::<TranslationJob>()?;
    m.add_class::<parse_module::high_level_api::ParseConfig>()?;
    m.add_class::<parse_module::high_level_api::FetchConfig>()?;
    m.add_class::<parse_module::high_level_api::ParseResult>()?;
    m.add_class::<parse_module::high_level_api::RustAssemblyParser>()?;

    // Add constants
    m.add("MATE_JOINER", parse_module::MATE_JOINER)?;
    m.add("SUBASSEMBLY_JOINER", parse_module::SUBASSEMBLY_JOINER)?;
    m.add("CHILD", parse_module::CHILD)?;
    m.add("PARENT", parse_module::PARENT)?;
    m.add("RELATION_CHILD", parse_module::RELATION_CHILD)?;
    m.add("RELATION_PARENT", parse_module::RELATION_PARENT)?;

    Ok(())
}
