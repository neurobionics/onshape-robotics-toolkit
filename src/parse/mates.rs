// Placeholder for mate processing functionality
// This module will be implemented in Phase 2

use std::collections::HashMap;
use crate::parse::models::*;
use crate::parse::{MATE_JOINER, SUBASSEMBLY_JOINER};

/// Create a part from a rigid subassembly with proper transformation handling
pub fn create_rigid_subassembly_part(
    assembly_key: &str,
    rigid_subassembly: &RootAssembly,
    rigid_assembly_to_part_tf: HashMap<String, MatedCS>,
) -> Part {
    Part {
        base: BaseMetadata {
            document_id: "rigid_assembly".to_string(),
            document_microversion: "microversion".to_string(),
            element_id: "element".to_string(),
            full_configuration: "default".to_string(),
            configuration: "default".to_string(),
        },
        part_id: assembly_key.to_string(),
        body_type: "solid".to_string(),
        is_standard_content: false,
        mass_property: rigid_subassembly.mass_property.clone(),
        document_version: None,
        is_rigid_assembly: Some(true),
        rigid_assembly_to_part_tf: Some(rigid_assembly_to_part_tf),
        rigid_assembly_workspace_id: Some("workspace_id".to_string()),
    }
}

/// Process assembly features to extract mates and relations
pub fn process_assembly_features(
    features: &[AssemblyFeature],
    parts: &mut HashMap<String, Part>,
    id_to_name_map: &HashMap<String, String>,
    rigid_subassembly_occurrence_map: &HashMap<String, HashMap<String, Occurrence>>,
    rigid_subassemblies: &HashMap<String, RootAssembly>,
    subassembly_prefix: Option<&str>,
) -> Result<(HashMap<String, MateFeatureData>, HashMap<String, MateRelationFeatureData>), String> {
    let mut mates_map = HashMap::new();
    let mut relations_map = HashMap::new();

    for feature in features {
        if feature.suppressed {
            continue;
        }

        match feature.feature_type {
            AssemblyFeatureType::Mate => {
                if let Some(mut mate_data) = feature.get_mate_data() {
                    if mate_data.mated_entities.len() < 2 {
                        log::warn!("Invalid mate feature: insufficient mated entities");
                        continue;
                    }

                    // Process mate data
                    process_mate_feature(
                        &mut mate_data,
                        feature,
                        parts,
                        id_to_name_map,
                        rigid_subassembly_occurrence_map,
                        rigid_subassemblies,
                        subassembly_prefix,
                        &mut mates_map,
                    )?;
                }
            }
            AssemblyFeatureType::MateRelation => {
                if let Some(mut relation_data) = feature.get_mate_relation_data() {
                    // Process mate relation data
                    process_mate_relation_feature(
                        &mut relation_data,
                        feature,
                        &mut relations_map,
                    )?;
                }
            }
            _ => {
                // Handle other feature types as needed
                log::debug!("Skipping unsupported feature type: {:?}", feature.feature_type);
            }
        }
    }

    Ok((mates_map, relations_map))
}

/// Process a single mate feature
fn process_mate_feature(
    mate_data: &mut MateFeatureData,
    feature: &AssemblyFeature,
    parts: &mut HashMap<String, Part>,
    id_to_name_map: &HashMap<String, String>,
    rigid_subassembly_occurrence_map: &HashMap<String, HashMap<String, Occurrence>>,
    rigid_subassemblies: &HashMap<String, RootAssembly>,
    subassembly_prefix: Option<&str>,
    mates_map: &mut HashMap<String, MateFeatureData>,
) -> Result<(), String> {
    // Set the ID from the feature
    mate_data.id = Some(feature.id.clone());

    // Get occurrence paths
    let child_occurrences: Vec<String> = mate_data.mated_entities[0].mated_occurrence
        .iter()
        .filter_map(|path| id_to_name_map.get(path).cloned())
        .collect();

    let parent_occurrences: Vec<String> = mate_data.mated_entities[1].mated_occurrence
        .iter()
        .filter_map(|path| id_to_name_map.get(path).cloned())
        .collect();

    if child_occurrences.is_empty() || parent_occurrences.is_empty() {
        return Err("Invalid occurrence paths in mate feature".to_string());
    }

    // Process rigid subassemblies
    let mut processed_parent = parent_occurrences;
    let mut processed_child = child_occurrences;

    // Handle parent rigid subassembly
    if let Some(parent_key) = processed_parent.first() {
        if rigid_subassemblies.contains_key(parent_key) && processed_parent.len() > 1 {
            if let Some(sub_occurrence_key) = processed_parent.get(1) {
                if let Some(occurrence_map) = rigid_subassembly_occurrence_map.get(parent_key) {
                    if let Some(occurrence) = occurrence_map.get(sub_occurrence_key) {
                        // Create transformation matrix
                        let transform_matrix = create_transform_matrix(&occurrence.transform)?;
                        let parent_cs = MatedCS::from_matrix(&transform_matrix);

                        // Update the part's transformation mapping
                        if let Some(part) = parts.get_mut(parent_key) {
                            if let Some(ref mut rigid_tf_map) = part.rigid_assembly_to_part_tf {
                                rigid_tf_map.insert(sub_occurrence_key.clone(), parent_cs.clone());
                            }
                        }

                        // Update the mate entity
                        mate_data.mated_entities[1].parent_cs = Some(parent_cs);
                    }
                }
            }
            processed_parent = vec![parent_key.clone()];
        }
    }

    // Handle child rigid subassembly
    if let Some(child_key) = processed_child.first() {
        if rigid_subassemblies.contains_key(child_key) && processed_child.len() > 1 {
            if let Some(sub_occurrence_key) = processed_child.get(1) {
                if let Some(occurrence_map) = rigid_subassembly_occurrence_map.get(child_key) {
                    if let Some(occurrence) = occurrence_map.get(sub_occurrence_key) {
                        // Create transformation matrix
                        let transform_matrix = create_transform_matrix(&occurrence.transform)?;
                        let child_cs = MatedCS::from_matrix(&transform_matrix);

                        // Update the part's transformation mapping
                        if let Some(part) = parts.get_mut(child_key) {
                            if let Some(ref mut rigid_tf_map) = part.rigid_assembly_to_part_tf {
                                rigid_tf_map.insert(sub_occurrence_key.clone(), child_cs.clone());
                            }
                        }

                        // Update the mate entity
                        mate_data.mated_entities[0].parent_cs = Some(child_cs);
                    }
                }
            }
            processed_child = vec![child_key.clone()];
        }
    }

    // Create the mate key
    let mate_key = join_mate_occurrences(
        &processed_parent,
        &processed_child,
        subassembly_prefix,
    );

    mates_map.insert(mate_key, mate_data.clone());
    Ok(())
}

/// Process a single mate relation feature
fn process_mate_relation_feature(
    relation_data: &mut MateRelationFeatureData,
    feature: &AssemblyFeature,
    relations_map: &mut HashMap<String, MateRelationFeatureData>,
) -> Result<(), String> {
    // Set the ID from the feature
    relation_data.id = Some(feature.id.clone());

    // Determine child joint ID based on relation type
    let child_joint_id = match relation_data.relation_type {
        RelationType::Screw => {
            if let Some(first_mate) = relation_data.mates.first() {
                first_mate.feature_id.clone()
            } else {
                return Err("Screw relation missing mates".to_string());
            }
        }
        _ => {
            if relation_data.mates.len() > 1 {
                relation_data.mates[1].feature_id.clone()
            } else {
                return Err("Relation missing child mate".to_string());
            }
        }
    };

    relations_map.insert(child_joint_id, relation_data.clone());
    Ok(())
}

/// Helper function to create transformation matrix from transform vector
fn create_transform_matrix(transform: &[f64]) -> Result<[f64; 16], String> {
    if transform.len() != 16 {
        return Err("Transform must have 16 elements".to_string());
    }

    let mut matrix = [0.0; 16];
    matrix.copy_from_slice(transform);
    Ok(matrix)
}

/// Join occurrence paths with mate joiner
fn join_mate_occurrences(
    parent: &[String],
    child: &[String],
    prefix: Option<&str>,
) -> String {
    let parent_occurrence = get_occurrence_name(parent, prefix);
    let child_occurrence = get_occurrence_name(child, prefix);
    format!("{}{}{}", parent_occurrence, MATE_JOINER, child_occurrence)
}

/// Get occurrence name with optional prefix
fn get_occurrence_name(occurrences: &[String], subassembly_prefix: Option<&str>) -> String {
    let prefix = if let Some(prefix) = subassembly_prefix {
        format!("{}{}", prefix, SUBASSEMBLY_JOINER)
    } else {
        String::new()
    };
    format!("{}{}", prefix, occurrences.join(SUBASSEMBLY_JOINER))
}

/// Validate coordinate system data
fn validate_coordinate_system(cs: &MatedCS) -> Result<(), String> {
    if cs.origin.len() != 3 || cs.x_axis.len() != 3 || cs.y_axis.len() != 3 || cs.z_axis.len() != 3 {
        return Err("Coordinate system vectors must have 3 elements".to_string());
    }
    Ok(())
}

/// Get mate statistics for analysis
pub fn get_mate_statistics(mates: &HashMap<String, MateFeatureData>) -> MateStats {
    let mut stats = MateStats::default();

    for mate_data in mates.values() {
        match mate_data.mate_type {
            MateType::Fastened => stats.fastened_count += 1,
            MateType::Revolute => stats.revolute_count += 1,
            MateType::Slider => stats.slider_count += 1,
            MateType::Cylindrical => stats.cylindrical_count += 1,
            MateType::Planar => stats.planar_count += 1,
            MateType::Ball => stats.ball_count += 1,
            MateType::PinSlot => stats.pin_slot_count += 1,
            MateType::Parallel => stats.parallel_count += 1,
        }
    }

    stats.total_mates = mates.len();
    stats
}

/// Get relation statistics for analysis
pub fn get_relation_statistics(relations: &HashMap<String, MateRelationFeatureData>) -> RelationStats {
    let mut stats = RelationStats::default();

    for relation_data in relations.values() {
        match relation_data.relation_type {
            RelationType::Gear => stats.gear_relation_count += 1,
            RelationType::Linear => stats.linear_relation_count += 1,
            RelationType::Screw => stats.screw_relation_count += 1,
            RelationType::RackAndPinion => stats.rack_pinion_relation_count += 1,
        }
    }

    stats.total_relations = relations.len();
    stats
}

/// Statistics for mate analysis
#[derive(Debug, Clone, Default)]
pub struct MateStats {
    pub total_mates: usize,
    pub fastened_count: usize,
    pub revolute_count: usize,
    pub slider_count: usize,
    pub cylindrical_count: usize,
    pub planar_count: usize,
    pub ball_count: usize,
    pub pin_slot_count: usize,
    pub parallel_count: usize,
}

/// Statistics for relation analysis
#[derive(Debug, Clone, Default)]
pub struct RelationStats {
    pub total_relations: usize,
    pub gear_relation_count: usize,
    pub linear_relation_count: usize,
    pub screw_relation_count: usize,
    pub rack_pinion_relation_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_transform_matrix() {
        let transform = vec![
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];

        let matrix = create_transform_matrix(&transform).unwrap();
        assert_eq!(matrix[0], 1.0);
        assert_eq!(matrix[5], 1.0);
        assert_eq!(matrix[10], 1.0);
        assert_eq!(matrix[15], 1.0);
    }

    #[test]
    fn test_get_occurrence_name() {
        let occurrences = vec!["part1".to_string(), "part2".to_string()];
        let name = get_occurrence_name(&occurrences, Some("sub1"));
        assert_eq!(name, "sub1_SUB_part1_SUB_part2");
    }
}
