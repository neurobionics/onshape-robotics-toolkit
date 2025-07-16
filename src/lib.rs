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
fn native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Existing bindings
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

    // New parse module bindings
    m.add_class::<parse::models::InstanceType>()?;
    m.add_class::<parse::models::MateType>()?;
    m.add_class::<parse::models::RelationType>()?;
    m.add_class::<parse::models::AssemblyFeatureType>()?;
    m.add_class::<parse::models::Occurrence>()?;
    m.add_class::<parse::models::IdBase>()?;
    m.add_class::<parse::models::MatedCS>()?;
    m.add_class::<parse::models::MatedEntity>()?;
    m.add_class::<parse::models::MateRelationMate>()?;
    m.add_class::<parse::models::MateRelationFeatureData>()?;
    m.add_class::<parse::models::MateFeatureData>()?;
    m.add_class::<parse::models::AssemblyFeature>()?;
    m.add_class::<parse::models::Part>()?;
    m.add_class::<parse::models::PartInstance>()?;
    m.add_class::<parse::models::AssemblyInstance>()?;
    m.add_class::<parse::models::SubAssembly>()?;
    m.add_class::<parse::models::RootAssembly>()?;
    m.add_class::<parse::models::Assembly>()?;

    Ok(())
}
