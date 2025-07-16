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
#[pyclass]
pub struct Variable {
    #[serde(rename = "type")]
    #[pyo3(get)]
    pub var_type: String,
    #[pyo3(get)]
    pub name: String,
    // Note: value is excluded from Python exposure due to complex JSON structure
    pub value: Option<serde_json::Value>,
    #[pyo3(get)]
    pub description: Option<String>,
    #[pyo3(get)]
    pub expression: String,
}

#[pymethods]
impl Variable {
    #[getter]
    pub fn value(&self) -> Option<String> {
        self.value.as_ref().map(|v| v.to_string())
    }
}

impl HasKey for Variable {
    fn key(&self) -> String {
        self.name.clone()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Assembly {
    #[serde(rename = "rootAssembly")]
    #[pyo3(get)]
    pub root_assembly: RootAssembly,
    #[serde(rename = "subAssemblies")]
    // Note: complex JSON arrays excluded from Python exposure
    pub sub_assemblies: Vec<serde_json::Value>,
    // Note: complex JSON arrays excluded from Python exposure
    pub parts: Vec<serde_json::Value>,
    #[serde(rename = "partStudioFeatures")]
    // Note: complex JSON arrays excluded from Python exposure
    pub part_studio_features: Vec<serde_json::Value>,
    #[serde(skip)]
    #[pyo3(get)]
    pub document: Option<Document>,
    #[serde(skip)]
    #[pyo3(get)]
    pub name: Option<String>,
}

#[pymethods]
impl Assembly {
    #[getter]
    pub fn sub_assemblies(&self) -> Vec<String> {
        self.sub_assemblies.iter().map(|v| v.to_string()).collect()
    }

    #[getter]
    pub fn parts(&self) -> Vec<String> {
        self.parts.iter().map(|v| v.to_string()).collect()
    }

    #[getter]
    pub fn part_studio_features(&self) -> Vec<String> {
        self.part_studio_features.iter().map(|v| v.to_string()).collect()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct RootAssembly {
    // Note: complex JSON arrays excluded from Python exposure
    pub instances: Vec<serde_json::Value>,
    pub patterns: Vec<serde_json::Value>,
    pub features: Vec<serde_json::Value>,
    pub occurrences: Vec<serde_json::Value>,
    #[serde(rename = "fullConfiguration")]
    #[pyo3(get)]
    pub full_configuration: String,
    #[pyo3(get)]
    pub configuration: String,
    #[serde(rename = "documentId")]
    #[pyo3(get)]
    pub document_id: String,
    #[serde(rename = "elementId")]
    #[pyo3(get)]
    pub element_id: String,
    #[serde(rename = "documentMicroversion")]
    #[pyo3(get)]
    pub document_microversion: String,
    #[serde(skip)]
    #[pyo3(get)]
    pub mass_property: Option<MassProperties>,
    #[serde(skip)]
    #[pyo3(get)]
    pub document_meta_data: Option<DocumentMetaData>,
}

#[pymethods]
impl RootAssembly {
    #[getter]
    pub fn instances(&self) -> Vec<String> {
        self.instances.iter().map(|v| v.to_string()).collect()
    }

    #[getter]
    pub fn patterns(&self) -> Vec<String> {
        self.patterns.iter().map(|v| v.to_string()).collect()
    }

    #[getter]
    pub fn features(&self) -> Vec<String> {
        self.features.iter().map(|v| v.to_string()).collect()
    }

    #[getter]
    pub fn occurrences(&self) -> Vec<String> {
        self.occurrences.iter().map(|v| v.to_string()).collect()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Document {
    #[pyo3(get)]
    pub did: String,
    #[pyo3(get)]
    pub wtype: String,
    #[pyo3(get)]
    pub wid: String,
    #[pyo3(get)]
    pub eid: String,
    #[serde(skip)]
    #[pyo3(get)]
    pub name: Option<String>,
    #[serde(skip)]
    #[pyo3(get)]
    pub url: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MassProperties {
    #[pyo3(get)]
    pub volume: Vec<f64>,
    #[pyo3(get)]
    pub mass: Vec<f64>,
    #[pyo3(get)]
    pub centroid: Vec<f64>,
    #[pyo3(get)]
    pub inertia: Vec<f64>,
    #[serde(rename = "principalInertia")]
    #[pyo3(get)]
    pub principal_inertia: Vec<f64>,
    #[serde(rename = "principalAxes")]
    #[pyo3(get)]
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
#[pyclass]
pub struct TranslationJob {
    #[pyo3(get)]
    pub id: String,
    #[serde(rename = "requestState")]
    #[pyo3(get)]
    pub request_state: String,
    #[serde(rename = "resultExternalDataIds")]
    #[pyo3(get)]
    pub result_external_data_ids: Option<Vec<String>>,
}
