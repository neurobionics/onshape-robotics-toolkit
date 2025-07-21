use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use crate::model::MassProperties;

/// Instance type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum InstanceType {
    Part,
    Assembly,
}

/// Mate type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum MateType {
    Slider,
    Cylindrical,
    Revolute,
    PinSlot,
    Planar,
    Ball,
    Fastened,
    Parallel,
}

/// Relation type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum RelationType {
    Linear,
    Gear,
    Screw,
    RackAndPinion,
}

/// Assembly feature type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum AssemblyFeatureType {
    Mate,
    MateRelation,
    MateGroup,
    MateConnector,
}

/// Transform matrix representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transform {
    pub matrix: Vec<f64>, // 4x4 transformation matrix as flat array
}

impl Transform {
    pub fn new(matrix: Vec<f64>) -> Self {
        Self { matrix }
    }

    pub fn identity() -> Self {
        Self {
            matrix: vec![
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }
}

/// Coordinate system representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatedCS {
    pub origin: Vec<f64>,
    pub x_axis: Vec<f64>,
    pub y_axis: Vec<f64>,
    pub z_axis: Vec<f64>,
}

impl MatedCS {
    pub fn new(origin: Vec<f64>, x_axis: Vec<f64>, y_axis: Vec<f64>, z_axis: Vec<f64>) -> Self {
        Self { origin, x_axis, y_axis, z_axis }
    }

    pub fn from_matrix(matrix: &[f64; 16]) -> Self {
        // Extract coordinate system from 4x4 transformation matrix
        let x_axis = vec![matrix[0], matrix[1], matrix[2]];
        let y_axis = vec![matrix[4], matrix[5], matrix[6]];
        let z_axis = vec![matrix[8], matrix[9], matrix[10]];
        let origin = vec![matrix[12], matrix[13], matrix[14]];

        Self { origin, x_axis, y_axis, z_axis }
    }
}

/// Occurrence path representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Occurrence {
    pub path: Vec<String>,
    pub transform: Vec<f64>, // 4x4 transformation matrix
    pub hidden: bool,
    pub suppressed: bool,
}

impl Occurrence {
    pub fn new(path: Vec<String>, transform: Vec<f64>, hidden: bool, suppressed: bool) -> Self {
        Self { path, transform, hidden, suppressed }
    }
}

/// Base metadata for assembly components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaseMetadata {
    pub document_id: String,
    pub document_microversion: String,
    pub element_id: String,
    pub full_configuration: String,
    pub configuration: String,
}

impl BaseMetadata {
    pub fn new(
        document_id: String,
        document_microversion: String,
        element_id: String,
        full_configuration: String,
        configuration: String,
    ) -> Self {
        Self {
            document_id,
            document_microversion,
            element_id,
            full_configuration,
            configuration,
        }
    }

    pub fn uid(&self) -> String {
        format!("{}:{}", self.document_id, self.element_id)
    }
}

/// Mated entity representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatedEntity {
    pub mated_occurrence: Vec<String>,
    pub parent_cs: Option<MatedCS>,
    pub mated_cs: MatedCS, // Add this field for compatibility
}

impl MatedEntity {
    pub fn new(mated_occurrence: Vec<String>, parent_cs: Option<MatedCS>, mated_cs: MatedCS) -> Self {
        Self { mated_occurrence, parent_cs, mated_cs }
    }
}

/// Feature reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureReference {
    pub feature_id: String,
    pub feature_type: String,
}

impl FeatureReference {
    pub fn new(feature_id: String, feature_type: String) -> Self {
        Self { feature_id, feature_type }
    }
}

/// Mate feature data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MateFeatureData {
    pub id: Option<String>,
    pub mated_entities: Vec<MatedEntity>,
    pub mate_type: MateType,
    pub name: String,
}

impl MateFeatureData {
    pub fn new(id: Option<String>, mated_entities: Vec<MatedEntity>, mate_type: MateType, name: String) -> Self {
        Self { id, mated_entities, mate_type, name }
    }
}

/// Mate relation feature data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MateRelationFeatureData {
    pub id: Option<String>,
    pub relation_type: RelationType,
    pub mates: Vec<FeatureReference>,
    pub relation_ratio: Option<f64>,
    pub relation_length: Option<f64>,
    pub name: String,
}

impl MateRelationFeatureData {
    pub fn new(
        id: Option<String>,
        relation_type: RelationType,
        mates: Vec<FeatureReference>,
        relation_ratio: Option<f64>,
        relation_length: Option<f64>,
        name: String,
    ) -> Self {
        Self { id, relation_type, mates, relation_ratio, relation_length, name }
    }
}

/// Assembly feature data variants
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AssemblyFeatureData {
    Mate(MateFeatureData),
    MateRelation(MateRelationFeatureData),
    Generic(serde_json::Value), // For other feature types
}

/// Assembly feature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssemblyFeature {
    pub id: String,
    pub feature_type: AssemblyFeatureType,
    pub suppressed: bool,
    pub feature_data: AssemblyFeatureData,
}

impl AssemblyFeature {
    pub fn new(
        id: String,
        feature_type: AssemblyFeatureType,
        suppressed: bool,
        feature_data: AssemblyFeatureData,
    ) -> Self {
        Self { id, feature_type, suppressed, feature_data }
    }

    pub fn get_mate_data(&self) -> Option<MateFeatureData> {
        match &self.feature_data {
            AssemblyFeatureData::Mate(data) => Some(data.clone()),
            _ => None,
        }
    }

    pub fn get_mate_relation_data(&self) -> Option<MateRelationFeatureData> {
        match &self.feature_data {
            AssemblyFeatureData::MateRelation(data) => Some(data.clone()),
            _ => None,
        }
    }
}

/// Part representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Part {
    pub base: BaseMetadata,
    pub part_id: String,
    pub body_type: String,
    pub is_standard_content: bool,
    pub mass_property: Option<MassProperties>,
    pub document_version: Option<String>,
    pub is_rigid_assembly: Option<bool>,
    pub rigid_assembly_to_part_tf: Option<HashMap<String, MatedCS>>,
    pub rigid_assembly_workspace_id: Option<String>,
}

impl Part {
    pub fn new(
        base: BaseMetadata,
        part_id: String,
        body_type: String,
        is_standard_content: bool,
        mass_property: Option<MassProperties>,
    ) -> Self {
        Self {
            base,
            part_id,
            body_type,
            is_standard_content,
            mass_property,
            document_version: None,
            is_rigid_assembly: None,
            rigid_assembly_to_part_tf: None,
            rigid_assembly_workspace_id: None,
        }
    }

    pub fn uid(&self) -> String {
        self.base.uid()
    }
}

/// Part instance representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartInstance {
    pub id: String,
    pub name: String,
    pub part_id: String,
    pub instance_type: InstanceType,
    pub suppressed: bool,
    pub hidden: bool,
    pub transform: Vec<f64>, // 4x4 transformation matrix
    pub is_standard_content: bool,
    pub document_version: Option<String>,
}

impl PartInstance {
    pub fn new(
        id: String,
        name: String,
        part_id: String,
        instance_type: InstanceType,
        suppressed: bool,
        hidden: bool,
        transform: Vec<f64>,
    ) -> Self {
        Self {
            id,
            name,
            part_id,
            instance_type,
            suppressed,
            hidden,
            transform,
            is_standard_content: false,
            document_version: None,
        }
    }

    pub fn uid(&self) -> String {
        self.part_id.clone()
    }
}

/// Assembly instance representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssemblyInstance {
    pub id: String,
    pub name: String,
    pub document_id: String,
    pub element_id: String,
    pub instance_type: InstanceType,
    pub suppressed: bool,
    pub hidden: bool,
    pub transform: Vec<f64>, // 4x4 transformation matrix
    pub is_rigid: Option<bool>,
}

impl AssemblyInstance {
    pub fn new(
        id: String,
        name: String,
        document_id: String,
        element_id: String,
        instance_type: InstanceType,
        suppressed: bool,
        hidden: bool,
        transform: Vec<f64>,
        is_rigid: Option<bool>,
    ) -> Self {
        Self {
            id,
            name,
            document_id,
            element_id,
            instance_type,
            suppressed,
            hidden,
            transform,
            is_rigid,
        }
    }

    pub fn uid(&self) -> String {
        format!("{}:{}", self.document_id, self.element_id)
    }
}

/// Instance variants (union type)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Instance {
    Part(PartInstance),
    Assembly(AssemblyInstance),
}

impl Instance {
    pub fn get_id(&self) -> &str {
        match self {
            Instance::Part(part) => &part.id,
            Instance::Assembly(assembly) => &assembly.id,
        }
    }

    pub fn get_name(&self) -> &str {
        match self {
            Instance::Part(part) => &part.name,
            Instance::Assembly(assembly) => &assembly.name,
        }
    }

    pub fn get_type(&self) -> &InstanceType {
        match self {
            Instance::Part(part) => &part.instance_type,
            Instance::Assembly(assembly) => &assembly.instance_type,
        }
    }

    pub fn uid(&self) -> String {
        match self {
            Instance::Part(part) => part.uid(),
            Instance::Assembly(assembly) => assembly.uid(),
        }
    }
}

/// SubAssembly representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubAssembly {
    pub base: BaseMetadata,
    pub instances: Vec<Instance>,
    pub features: Vec<AssemblyFeature>,
}

impl SubAssembly {
    pub fn new(
        base: BaseMetadata,
        instances: Vec<Instance>,
        features: Vec<AssemblyFeature>,
    ) -> Self {
        Self { base, instances, features }
    }

    pub fn uid(&self) -> String {
        self.base.uid()
    }
}

/// Root assembly representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RootAssembly {
    pub sub_assembly: SubAssembly,
    pub occurrences: Vec<Occurrence>,
    pub mass_property: Option<MassProperties>,
}

impl RootAssembly {
    pub fn new(
        sub_assembly: SubAssembly,
        occurrences: Vec<Occurrence>,
        mass_property: Option<MassProperties>,
    ) -> Self {
        Self { sub_assembly, occurrences, mass_property }
    }
}

/// Complete assembly representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Assembly {
    pub root_assembly: RootAssembly,
    pub sub_assemblies: Vec<SubAssembly>,
    pub parts: Vec<Part>,
}

impl Assembly {
    pub fn new(
        root_assembly: RootAssembly,
        sub_assemblies: Vec<SubAssembly>,
        parts: Vec<Part>,
    ) -> Self {
        Self { root_assembly, sub_assemblies, parts }
    }
}

// Conversion from client model to parse model
impl From<crate::model::RootAssembly> for RootAssembly {
    fn from(root: crate::model::RootAssembly) -> Self {
        // Convert instances to our format
        let instances: Vec<Instance> = root.instances
            .into_iter()
            .filter_map(|json_value| {
                // Clone for logging if parsing fails
                let json_value_clone = json_value.clone();

                // Try to parse as PartInstance first
                if let Ok(part_instance) = serde_json::from_value::<serde_json::Value>(json_value.clone())
                    .and_then(|val| {
                        if val.get("type").and_then(|t| t.as_str()) == Some("Part") {
                            Ok(PartInstance {
                                id: val.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                name: val.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                part_id: val.get("partId").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                instance_type: InstanceType::Part,
                                suppressed: val.get("suppressed").and_then(|v| v.as_bool()).unwrap_or(false),
                                hidden: val.get("hidden").and_then(|v| v.as_bool()).unwrap_or(false),
                                transform: val.get("transform").and_then(|v| v.as_array())
                                    .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                    .unwrap_or_else(|| vec![1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]),
                                is_standard_content: val.get("isStandardContent").and_then(|v| v.as_bool()).unwrap_or(false),
                                document_version: val.get("documentVersion").and_then(|v| v.as_str()).map(|s| s.to_string()),
                            })
                        } else {
                            Err(serde_json::Error::io(std::io::Error::new(
                                std::io::ErrorKind::InvalidData,
                                "Not a part instance"
                            )))
                        }
                    })
                {
                    Some(Instance::Part(part_instance))
                } else if let Ok(assembly_instance) = serde_json::from_value::<serde_json::Value>(json_value)
                    .and_then(|val| {
                        if val.get("type").and_then(|t| t.as_str()) == Some("Assembly") {
                            Ok(AssemblyInstance {
                                id: val.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                name: val.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                document_id: val.get("documentId").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                element_id: val.get("elementId").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                instance_type: InstanceType::Assembly,
                                suppressed: val.get("suppressed").and_then(|v| v.as_bool()).unwrap_or(false),
                                hidden: val.get("hidden").and_then(|v| v.as_bool()).unwrap_or(false),
                                transform: val.get("transform").and_then(|v| v.as_array())
                                    .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                    .unwrap_or_else(|| vec![1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]),
                                is_rigid: None,
                            })
                        } else {
                            Err(serde_json::Error::io(std::io::Error::new(
                                std::io::ErrorKind::InvalidData,
                                "Not an assembly instance"
                            )))
                        }
                    })
                {
                    Some(Instance::Assembly(assembly_instance))
                } else {
                    log::warn!("Failed to parse instance: {:?}", json_value_clone);
                    None
                }
            })
            .collect();

        // Convert features to our format
        let features: Vec<AssemblyFeature> = root.features
            .into_iter()
            .filter_map(|json_value| {
                serde_json::from_value::<serde_json::Value>(json_value.clone())
                    .ok()
                    .and_then(|val| {
                        let id = val.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string();
                        let suppressed = val.get("suppressed").and_then(|v| v.as_bool()).unwrap_or(false);
                        let feature_type_str = val.get("featureType").and_then(|v| v.as_str()).unwrap_or("");

                        let feature_type = match feature_type_str {
                            "mate" => AssemblyFeatureType::Mate,
                            "mateRelation" => AssemblyFeatureType::MateRelation,
                            "mateGroup" => AssemblyFeatureType::MateGroup,
                            "mateConnector" => AssemblyFeatureType::MateConnector,
                            _ => return None,
                        };

                        let feature_data = if let Some(feature_data_val) = val.get("featureData") {
                            match feature_type {
                                AssemblyFeatureType::Mate => {
                                    // Parse mate feature data
                                    let mate_type_str = feature_data_val.get("mateType").and_then(|v| v.as_str()).unwrap_or("");
                                    let mate_type = match mate_type_str {
                                        "FASTENED" => MateType::Fastened,
                                        "REVOLUTE" => MateType::Revolute,
                                        "SLIDER" => MateType::Slider,
                                        "CYLINDRICAL" => MateType::Cylindrical,
                                        "PLANAR" => MateType::Planar,
                                        "BALL" => MateType::Ball,
                                        "PIN_SLOT" => MateType::PinSlot,
                                        "PARALLEL" => MateType::Parallel,
                                        _ => return None,
                                    };

                                    let mated_entities = feature_data_val.get("matedEntities")
                                        .and_then(|v| v.as_array())
                                        .map(|arr| {
                                            arr.iter().filter_map(|entity_val| {
                                                let mated_occurrence = entity_val.get("matedOccurrence")
                                                    .and_then(|v| v.as_array())
                                                    .map(|arr| arr.iter().filter_map(|v| v.as_str().map(|s| s.to_string())).collect())
                                                    .unwrap_or_default();

                                                // Create a basic MatedCS from available data
                                                let mated_cs = entity_val.get("matedCS")
                                                    .map(|cs_val| MatedCS {
                                                        origin: cs_val.get("origin").and_then(|v| v.as_array())
                                                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                                            .unwrap_or_else(|| vec![0.0, 0.0, 0.0]),
                                                        x_axis: cs_val.get("xAxis").and_then(|v| v.as_array())
                                                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                                            .unwrap_or_else(|| vec![1.0, 0.0, 0.0]),
                                                        y_axis: cs_val.get("yAxis").and_then(|v| v.as_array())
                                                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                                            .unwrap_or_else(|| vec![0.0, 1.0, 0.0]),
                                                        z_axis: cs_val.get("zAxis").and_then(|v| v.as_array())
                                                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                                            .unwrap_or_else(|| vec![0.0, 0.0, 1.0]),
                                                    })
                                                    .unwrap_or_else(|| MatedCS {
                                                        origin: vec![0.0, 0.0, 0.0],
                                                        x_axis: vec![1.0, 0.0, 0.0],
                                                        y_axis: vec![0.0, 1.0, 0.0],
                                                        z_axis: vec![0.0, 0.0, 1.0],
                                                    });

                                                Some(MatedEntity {
                                                    mated_occurrence,
                                                    parent_cs: None,
                                                    mated_cs,
                                                })
                                            }).collect()
                                        })
                                        .unwrap_or_default();

                                    AssemblyFeatureData::Mate(MateFeatureData {
                                        id: None,
                                        mated_entities,
                                        mate_type,
                                        name: feature_data_val.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                    })
                                }
                                AssemblyFeatureType::MateRelation => {
                                    let relation_type_str = feature_data_val.get("relationType").and_then(|v| v.as_str()).unwrap_or("");
                                    let relation_type = match relation_type_str {
                                        "LINEAR" => RelationType::Linear,
                                        "GEAR" => RelationType::Gear,
                                        "SCREW" => RelationType::Screw,
                                        "RACK_AND_PINION" => RelationType::RackAndPinion,
                                        _ => return None,
                                    };

                                    let mates = feature_data_val.get("mates")
                                        .and_then(|v| v.as_array())
                                        .map(|arr| {
                                            arr.iter().filter_map(|mate_val| {
                                                Some(FeatureReference {
                                                    feature_id: mate_val.get("featureId").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                                    feature_type: mate_val.get("featureType").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                                })
                                            }).collect()
                                        })
                                        .unwrap_or_default();

                                    AssemblyFeatureData::MateRelation(MateRelationFeatureData {
                                        id: None,
                                        relation_type,
                                        mates,
                                        relation_ratio: feature_data_val.get("relationRatio").and_then(|v| v.as_f64()),
                                        relation_length: feature_data_val.get("relationLength").and_then(|v| v.as_f64()),
                                        name: feature_data_val.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                    })
                                }
                                _ => {
                                    // Generic feature data for unsupported types
                                    AssemblyFeatureData::Generic(feature_data_val.clone())
                                }
                            }
                        } else {
                            AssemblyFeatureData::Generic(serde_json::Value::Null)
                        };

                        Some(AssemblyFeature {
                            id,
                            feature_type,
                            suppressed,
                            feature_data,
                        })
                    })
            })
            .collect();

        // Convert occurrences to our format
        let occurrences: Vec<Occurrence> = root.occurrences
            .into_iter()
            .filter_map(|json_value| {
                serde_json::from_value::<serde_json::Value>(json_value)
                    .ok()
                    .and_then(|val| {
                        let path = val.get("path")
                            .and_then(|v| v.as_array())
                            .map(|arr| arr.iter().filter_map(|v| v.as_str().map(|s| s.to_string())).collect())
                            .unwrap_or_default();

                        let transform = val.get("transform")
                            .and_then(|v| v.as_array())
                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                            .unwrap_or_else(|| vec![1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]);

                        let hidden = val.get("hidden").and_then(|v| v.as_bool()).unwrap_or(false);
                        let suppressed = val.get("suppressed").and_then(|v| v.as_bool()).unwrap_or(false);

                        Some(Occurrence {
                            path,
                            transform,
                            hidden,
                            suppressed,
                        })
                    })
            })
            .collect();

        // Create the SubAssembly part
        let sub_assembly = SubAssembly {
            base: BaseMetadata {
                full_configuration: root.full_configuration,
                configuration: root.configuration,
                document_id: root.document_id,
                element_id: root.element_id,
                document_microversion: root.document_microversion,
            },
            instances,
            features,
        };

        RootAssembly {
            sub_assembly,
            occurrences,
            mass_property: root.mass_property,
        }
    }
}
