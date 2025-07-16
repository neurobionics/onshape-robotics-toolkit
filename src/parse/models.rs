use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use pyo3::prelude::*;
use nalgebra::Matrix4;
use crate::model::{MassProperties, DocumentMetaData, Document};

/// Enumerates the types of instances in an assembly
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[pyclass(eq, eq_int)]
pub enum InstanceType {
    #[serde(rename = "Part")]
    Part,
    #[serde(rename = "Assembly")]
    Assembly,
}

/// Enumerates the type of mate between two parts or assemblies
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[pyclass(eq, eq_int)]
pub enum MateType {
    #[serde(rename = "SLIDER")]
    Slider,
    #[serde(rename = "CYLINDRICAL")]
    Cylindrical,
    #[serde(rename = "REVOLUTE")]
    Revolute,
    #[serde(rename = "PIN_SLOT")]
    PinSlot,
    #[serde(rename = "PLANAR")]
    Planar,
    #[serde(rename = "BALL")]
    Ball,
    #[serde(rename = "FASTENED")]
    Fastened,
    #[serde(rename = "PARALLEL")]
    Parallel,
}

/// Enumerates the type of mate relation between two parts or assemblies
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[pyclass(eq, eq_int)]
pub enum RelationType {
    #[serde(rename = "LINEAR")]
    Linear,
    #[serde(rename = "GEAR")]
    Gear,
    #[serde(rename = "SCREW")]
    Screw,
    #[serde(rename = "RACK_AND_PINION")]
    RackAndPinion,
}

/// Enumerates the type of assembly feature
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[pyclass(eq, eq_int)]
pub enum AssemblyFeatureType {
    #[serde(rename = "mate")]
    Mate,
    #[serde(rename = "mateRelation")]
    MateRelation,
    #[serde(rename = "mateGroup")]
    MateGroup,
    #[serde(rename = "mateConnector")]
    MateConnector,
}

/// Represents an occurrence of a part or sub-assembly within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Occurrence {
    #[pyo3(get)]
    pub fixed: bool,
    #[pyo3(get)]
    pub transform: Vec<f64>, // 16-element transformation matrix
    #[pyo3(get)]
    pub hidden: bool,
    #[pyo3(get)]
    pub path: Vec<String>,
}

#[pymethods]
impl Occurrence {
    /// Get the transformation matrix as a 4x4 nalgebra matrix
    pub fn get_transform_matrix(&self) -> PyResult<Vec<Vec<f64>>> {
        if self.transform.len() != 16 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "Transform must have 16 elements"
            ));
        }

        let matrix = Matrix4::from_column_slice(&self.transform);
        Ok(matrix.data.as_slice().chunks(4).map(|row| row.to_vec()).collect())
    }
}

impl Occurrence {
    /// Get the transformation matrix as a nalgebra Matrix4
    pub fn transform_matrix(&self) -> Result<Matrix4<f64>, String> {
        if self.transform.len() != 16 {
            return Err("Transform must have 16 elements".to_string());
        }
        Ok(Matrix4::from_column_slice(&self.transform))
    }
}

/// Base model providing common attributes for Part, SubAssembly, and AssemblyInstance models
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct IdBase {
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
}

#[pymethods]
impl IdBase {
    /// Generate a unique identifier
    pub fn uid(&self) -> String {
        use uuid::Uuid;
        let input = format!("{}-{}-{}-{}",
            self.document_id,
            self.document_microversion,
            self.element_id,
            self.full_configuration
        );
        Uuid::new_v5(&Uuid::NAMESPACE_OID, input.as_bytes()).to_string()
    }
}

/// Represents a coordinate system used for mating parts within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MatedCS {
    #[serde(rename = "xAxis")]
    #[pyo3(get)]
    pub x_axis: Vec<f64>,
    #[serde(rename = "yAxis")]
    #[pyo3(get)]
    pub y_axis: Vec<f64>,
    #[serde(rename = "zAxis")]
    #[pyo3(get)]
    pub z_axis: Vec<f64>,
    #[pyo3(get)]
    pub origin: Vec<f64>,

    // Custom transformation matrix (not serialized)
    #[serde(skip)]
    pub part_tf: Option<Matrix4<f64>>,
}

#[pymethods]
impl MatedCS {
    #[new]
    pub fn new(x_axis: Vec<f64>, y_axis: Vec<f64>, z_axis: Vec<f64>, origin: Vec<f64>) -> PyResult<Self> {
        if x_axis.len() != 3 || y_axis.len() != 3 || z_axis.len() != 3 || origin.len() != 3 {
            return Err(pyo3::exceptions::PyValueError::new_err("All vectors must have 3 elements"));
        }

        Ok(MatedCS {
            x_axis,
            y_axis,
            z_axis,
            origin,
            part_tf: None,
        })
    }

    /// Get the part-to-mate transformation matrix
    pub fn part_to_mate_tf(&self) -> PyResult<Vec<Vec<f64>>> {
        let matrix = self.get_part_to_mate_matrix()
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e))?;
        Ok(matrix.data.as_slice().chunks(4).map(|row| row.to_vec()).collect())
    }
}

impl MatedCS {
    /// Get the part-to-mate transformation matrix as nalgebra Matrix4
    pub fn get_part_to_mate_matrix(&self) -> Result<Matrix4<f64>, String> {
        if let Some(tf) = &self.part_tf {
            return Ok(*tf);
        }

        if self.x_axis.len() != 3 || self.y_axis.len() != 3 || self.z_axis.len() != 3 || self.origin.len() != 3 {
            return Err("All vectors must have 3 elements".to_string());
        }

        let mut matrix = Matrix4::identity();

        // Set rotation part (column-major)
        matrix[(0, 0)] = self.x_axis[0]; matrix[(1, 0)] = self.x_axis[1]; matrix[(2, 0)] = self.x_axis[2];
        matrix[(0, 1)] = self.y_axis[0]; matrix[(1, 1)] = self.y_axis[1]; matrix[(2, 1)] = self.y_axis[2];
        matrix[(0, 2)] = self.z_axis[0]; matrix[(1, 2)] = self.z_axis[1]; matrix[(2, 2)] = self.z_axis[2];

        // Set translation part
        matrix[(0, 3)] = self.origin[0]; matrix[(1, 3)] = self.origin[1]; matrix[(2, 3)] = self.origin[2];

        Ok(matrix)
    }

    /// Create MatedCS from transformation matrix
    pub fn from_matrix(matrix: &Matrix4<f64>) -> Self {
        let x_axis = vec![matrix[(0, 0)], matrix[(1, 0)], matrix[(2, 0)]];
        let y_axis = vec![matrix[(0, 1)], matrix[(1, 1)], matrix[(2, 1)]];
        let z_axis = vec![matrix[(0, 2)], matrix[(1, 2)], matrix[(2, 2)]];
        let origin = vec![matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]];

        MatedCS {
            x_axis,
            y_axis,
            z_axis,
            origin,
            part_tf: Some(*matrix),
        }
    }
}

/// Represents an entity that is mated within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MatedEntity {
    #[serde(rename = "matedOccurrence")]
    #[pyo3(get)]
    pub mated_occurrence: Vec<String>,
    #[serde(rename = "matedCS")]
    #[pyo3(get)]
    pub mated_cs: MatedCS,
    #[serde(skip)]
    #[pyo3(get)]
    pub parent_cs: Option<MatedCS>,
}

/// Represents a mate relation within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MateRelationMate {
    #[serde(rename = "featureId")]
    #[pyo3(get)]
    pub feature_id: String,
    #[pyo3(get)]
    pub occurrence: Vec<String>,
}

/// Represents data for a mate relation feature within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MateRelationFeatureData {
    #[serde(rename = "relationType")]
    #[pyo3(get)]
    pub relation_type: RelationType,
    #[pyo3(get)]
    pub mates: Vec<MateRelationMate>,
    #[serde(rename = "reverseDirection")]
    #[pyo3(get)]
    pub reverse_direction: bool,
    #[serde(rename = "relationRatio")]
    #[pyo3(get)]
    pub relation_ratio: Option<f64>,
    #[serde(rename = "relationLength")]
    #[pyo3(get)]
    pub relation_length: Option<f64>,
    #[pyo3(get)]
    pub name: String,

    // Custom attribute set by processing
    #[serde(skip)]
    #[pyo3(get)]
    pub id: Option<String>,
}

/// Represents data for a mate feature within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct MateFeatureData {
    #[serde(rename = "matedEntities")]
    #[pyo3(get)]
    pub mated_entities: Vec<MatedEntity>,
    #[serde(rename = "mateType")]
    #[pyo3(get)]
    pub mate_type: MateType,
    #[pyo3(get)]
    pub name: String,

    // Custom attribute set by processing
    #[serde(skip)]
    #[pyo3(get)]
    pub id: Option<String>,
}

/// Union type for assembly feature data
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum AssemblyFeatureData {
    Mate(MateFeatureData),
    MateRelation(MateRelationFeatureData),
    // Add other feature types as needed
    Generic(serde_json::Value),
}

/// Represents a feature within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct AssemblyFeature {
    #[pyo3(get)]
    pub id: String,
    #[pyo3(get)]
    pub suppressed: bool,
    #[serde(rename = "featureType")]
    #[pyo3(get)]
    pub feature_type: AssemblyFeatureType,
    #[serde(rename = "featureData")]
    pub feature_data: AssemblyFeatureData,
}

#[pymethods]
impl AssemblyFeature {
    /// Get mate feature data if this is a mate feature
    pub fn get_mate_data(&self) -> Option<MateFeatureData> {
        match &self.feature_data {
            AssemblyFeatureData::Mate(data) => Some(data.clone()),
            _ => None,
        }
    }

    /// Get mate relation feature data if this is a mate relation feature
    pub fn get_mate_relation_data(&self) -> Option<MateRelationFeatureData> {
        match &self.feature_data {
            AssemblyFeatureData::MateRelation(data) => Some(data.clone()),
            _ => None,
        }
    }
}

/// Represents a part within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Part {
    // Inherits from IdBase
    #[serde(flatten)]
    #[pyo3(get)]
    pub base: IdBase,

    #[serde(rename = "isStandardContent")]
    #[pyo3(get)]
    pub is_standard_content: bool,
    #[serde(rename = "partId")]
    #[pyo3(get)]
    pub part_id: String,
    #[serde(rename = "bodyType")]
    #[pyo3(get)]
    pub body_type: String,
    #[serde(rename = "documentVersion")]
    #[pyo3(get)]
    pub document_version: Option<String>,

    // Custom attributes
    #[serde(skip)]
    #[pyo3(get)]
    pub mass_property: Option<MassProperties>,
    #[serde(skip)]
    #[pyo3(get)]
    pub is_rigid_assembly: Option<bool>,
    #[serde(skip)]
    pub rigid_assembly_to_part_tf: Option<HashMap<String, MatedCS>>,
    #[serde(skip)]
    #[pyo3(get)]
    pub rigid_assembly_workspace_id: Option<String>,
}

#[pymethods]
impl Part {
    /// Generate unique identifier for the part
    pub fn uid(&self) -> String {
        use uuid::Uuid;
        let input = format!("{}-{}-{}-{}-{}",
            self.base.document_id,
            self.base.document_microversion,
            self.base.element_id,
            self.part_id,
            self.base.full_configuration
        );
        Uuid::new_v5(&Uuid::NAMESPACE_OID, input.as_bytes()).to_string()
    }
}

/// Represents an instance of a part within an assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct PartInstance {
    // Inherits from IdBase
    #[serde(flatten)]
    #[pyo3(get)]
    pub base: IdBase,

    #[serde(rename = "isStandardContent")]
    #[pyo3(get)]
    pub is_standard_content: bool,
    #[serde(rename = "type")]
    #[pyo3(get)]
    pub instance_type: InstanceType,
    #[pyo3(get)]
    pub id: String,
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub suppressed: bool,
    #[serde(rename = "partId")]
    #[pyo3(get)]
    pub part_id: String,
    #[serde(rename = "documentVersion")]
    #[pyo3(get)]
    pub document_version: Option<String>,
}

#[pymethods]
impl PartInstance {
    /// Generate unique identifier for the part instance
    pub fn uid(&self) -> String {
        use uuid::Uuid;
        let input = format!("{}-{}-{}-{}-{}",
            self.base.document_id,
            self.base.document_microversion,
            self.base.element_id,
            self.part_id,
            self.base.full_configuration
        );
        Uuid::new_v5(&Uuid::NAMESPACE_OID, input.as_bytes()).to_string()
    }
}

/// Represents an instance of an assembly within another assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct AssemblyInstance {
    // Inherits from IdBase
    #[serde(flatten)]
    #[pyo3(get)]
    pub base: IdBase,

    #[pyo3(get)]
    pub id: String,
    #[serde(rename = "type")]
    #[pyo3(get)]
    pub instance_type: InstanceType,
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub suppressed: bool,

    // Custom attributes
    #[serde(skip)]
    #[pyo3(get)]
    pub is_rigid: Option<bool>,
}

#[pymethods]
impl AssemblyInstance {
    /// Generate unique identifier for the assembly instance
    pub fn uid(&self) -> String {
        self.base.uid()
    }
}

/// Union type for instances
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Instance {
    Part(PartInstance),
    Assembly(AssemblyInstance),
}

impl Instance {
    pub fn get_id(&self) -> &str {
        match self {
            Instance::Part(p) => &p.id,
            Instance::Assembly(a) => &a.id,
        }
    }

    pub fn get_name(&self) -> &str {
        match self {
            Instance::Part(p) => &p.name,
            Instance::Assembly(a) => &a.name,
        }
    }

    pub fn get_type(&self) -> &InstanceType {
        match self {
            Instance::Part(p) => &p.instance_type,
            Instance::Assembly(a) => &a.instance_type,
        }
    }

    pub fn uid(&self) -> String {
        match self {
            Instance::Part(p) => p.uid(),
            Instance::Assembly(a) => a.uid(),
        }
    }
}

/// Represents a sub-assembly within a larger assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct SubAssembly {
    // Inherits from IdBase
    #[serde(flatten)]
    #[pyo3(get)]
    pub base: IdBase,

    pub instances: Vec<Instance>,
    pub patterns: Vec<serde_json::Value>, // Keeping as JSON for now
    #[pyo3(get)]
    pub features: Vec<AssemblyFeature>,
}

#[pymethods]
impl SubAssembly {
    /// Generate unique identifier for the subassembly
    pub fn uid(&self) -> String {
        self.base.uid()
    }

    /// Get instances as strings for Python compatibility
    #[getter]
    pub fn instances(&self) -> Vec<String> {
        self.instances.iter().map(|i| format!("{:?}", i)).collect()
    }

    /// Get patterns as strings for Python compatibility
    #[getter]
    pub fn patterns(&self) -> Vec<String> {
        self.patterns.iter().map(|p| p.to_string()).collect()
    }
}

/// Represents the root assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct RootAssembly {
    // Inherits from SubAssembly
    #[serde(flatten)]
    #[pyo3(get)]
    pub sub_assembly: SubAssembly,

    #[pyo3(get)]
    pub occurrences: Vec<Occurrence>,

    // Custom attributes
    #[serde(skip)]
    #[pyo3(get)]
    pub mass_property: Option<MassProperties>,
    #[serde(skip)]
    #[pyo3(get)]
    pub document_meta_data: Option<DocumentMetaData>,
}

/// Represents the overall assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
#[pyclass]
pub struct Assembly {
    #[serde(rename = "rootAssembly")]
    #[pyo3(get)]
    pub root_assembly: RootAssembly,
    #[serde(rename = "subAssemblies")]
    #[pyo3(get)]
    pub sub_assemblies: Vec<SubAssembly>,
    #[pyo3(get)]
    pub parts: Vec<Part>,
    #[serde(rename = "partStudioFeatures")]
    pub part_studio_features: Vec<serde_json::Value>, // Keeping as JSON for now

    // Custom attributes
    #[serde(skip)]
    #[pyo3(get)]
    pub document: Option<Document>,
    #[serde(skip)]
    #[pyo3(get)]
    pub name: Option<String>,
}

#[pymethods]
impl Assembly {
    /// Get part studio features as strings for Python compatibility
    #[getter]
    pub fn part_studio_features(&self) -> Vec<String> {
        self.part_studio_features.iter().map(|f| f.to_string()).collect()
    }
}

// Conversion from client model to parse model
impl From<crate::model::RootAssembly> for RootAssembly {
    fn from(root: crate::model::RootAssembly) -> Self {
        // Convert JSON instances to typed instances
        let instances: Vec<Instance> = root.instances
            .into_iter()
            .filter_map(|json| serde_json::from_value(json).ok())
            .collect();

        // Convert JSON patterns to typed patterns
        let patterns: Vec<serde_json::Value> = root.patterns;

        // Convert JSON features to typed features
        let features: Vec<AssemblyFeature> = root.features
            .into_iter()
            .filter_map(|json| serde_json::from_value(json).ok())
            .collect();

        // Convert JSON occurrences to typed occurrences
        let occurrences: Vec<Occurrence> = root.occurrences
            .into_iter()
            .filter_map(|json| serde_json::from_value(json).ok())
            .collect();

        // Create the SubAssembly part
        let sub_assembly = SubAssembly {
            base: IdBase {
                full_configuration: root.full_configuration,
                configuration: root.configuration,
                document_id: root.document_id,
                element_id: root.element_id,
                document_microversion: root.document_microversion,
            },
            instances,
            patterns,
            features,
        };

        RootAssembly {
            sub_assembly,
            occurrences,
            mass_property: root.mass_property,
            document_meta_data: root.document_meta_data,
        }
    }
}
