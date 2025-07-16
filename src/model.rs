use serde::{Deserialize, Serialize};
use pyo3::prelude::*;

use crate::endpoint::HasKey;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct DocumentMetaData {
    #[pyo3(get)]
    pub id: String,
    #[pyo3(get)]
    pub name: String,
    #[serde(rename = "defaultWorkspace")]
    #[pyo3(get)]
    pub default_workspace: DefaultWorkspace,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct DefaultWorkspace {
    #[pyo3(get)]
    pub id: String,
    #[serde(rename = "type")]
    #[pyo3(get, set)]
    pub workspace_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Element {
    #[pyo3(get)]
    pub id: String,
    #[pyo3(get)]
    pub name: String,
    #[serde(rename = "elementType")]
    #[pyo3(get)]
    pub element_type: String,
    #[serde(rename = "microversionId")]
    #[pyo3(get)]
    pub microversion_id: String,
}

impl HasKey for Element {
    fn key(&self) -> String {
        self.name.clone()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Variable {
    #[serde(rename = "type")]
    pub var_type: String,
    pub name: String,
    pub value: Option<serde_json::Value>,
    pub description: Option<String>,
    pub expression: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Assembly {
    #[serde(rename = "rootAssembly")]
    pub root_assembly: RootAssembly,
    #[serde(rename = "subAssemblies")]
    pub sub_assemblies: Vec<serde_json::Value>,
    pub parts: Vec<serde_json::Value>,
    #[serde(rename = "partStudioFeatures")]
    pub part_studio_features: Vec<serde_json::Value>,
    #[serde(skip)]
    pub document: Option<Document>,
    #[serde(skip)]
    pub name: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RootAssembly {
    pub instances: Vec<serde_json::Value>,
    pub patterns: Vec<serde_json::Value>,
    pub features: Vec<serde_json::Value>,
    pub occurrences: Vec<serde_json::Value>,
    #[serde(rename = "fullConfiguration")]
    pub full_configuration: String,
    pub configuration: String,
    #[serde(rename = "documentId")]
    pub document_id: String,
    #[serde(rename = "elementId")]
    pub element_id: String,
    #[serde(rename = "documentMicroversion")]
    pub document_microversion: String,
    #[serde(skip)]
    pub mass_property: Option<MassProperties>,
    #[serde(skip)]
    pub document_meta_data: Option<DocumentMetaData>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Document {
    pub did: String,
    pub wtype: String,
    pub wid: String,
    pub eid: String,
    #[serde(skip)]
    pub name: Option<String>,
    #[serde(skip)]
    pub url: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MassProperties {
    pub volume: Vec<f64>,
    pub mass: Vec<f64>,
    pub centroid: Vec<f64>,
    pub inertia: Vec<f64>,
    #[serde(rename = "principalInertia")]
    pub principal_inertia: Vec<f64>,
    #[serde(rename = "principalAxes")]
    pub principal_axes: Vec<f64>,
}

/// Helper type for endpoints that accept name or ID
#[derive(Debug, Clone)]
pub enum NameOrId<'a> {
    Name(&'a str),
    Id(&'a str),
}

impl<'a> std::fmt::Display for NameOrId<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NameOrId::Name(name) => write!(f, "{}", name),
            NameOrId::Id(id) => write!(f, "{}", id),
        }
    }
}

impl<'a> From<&'a str> for NameOrId<'a> {
    fn from(s: &'a str) -> Self {
        NameOrId::Name(s)
    }
}

/// Translation job information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranslationJob {
    pub id: String,
    #[serde(rename = "requestState")]
    pub request_state: String,
    #[serde(rename = "resultExternalDataIds")]
    pub result_external_data_ids: Option<Vec<String>>,
}
