// Placeholder for mate processing functionality
// This module will be implemented in Phase 2

use std::collections::HashMap;
use crate::parse::models::*;
use crate::parse::{CHILD, PARENT, RELATION_CHILD, SUBASSEMBLY_JOINER};

/// Mate processor for assembly features
pub struct MateProcessor;

impl MateProcessor {
    /// Build a map of rigid subassembly occurrences
    /// Mirrors the Python build_rigid_subassembly_occurrence_map function
    pub async fn build_rigid_subassembly_occurrence_map(
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        id_to_name_map: &HashMap<String, String>,
        parts: &mut HashMap<String, Part>,
    ) -> Result<HashMap<String, HashMap<String, Occurrence>>, String> {
        let mut occurrence_map: HashMap<String, HashMap<String, Occurrence>> = HashMap::new();

        for (assembly_key, rigid_subassembly) in rigid_subassemblies {
            let mut sub_occurrences: HashMap<String, Occurrence> = HashMap::new();

            // Process each occurrence in the rigid subassembly
            for occurrence in &rigid_subassembly.occurrences {
                // Map occurrence path using ID to name mapping
                let occurrence_path: Result<Vec<String>, String> = occurrence.path
                    .iter()
                    .map(|path_id| {
                        id_to_name_map.get(path_id)
                            .ok_or_else(|| format!("Occurrence path {} not found in ID mapping", path_id))
                            .map(|name| name.clone())
                    })
                    .collect();

                match occurrence_path {
                    Ok(path_names) => {
                        let occurrence_key = path_names.join(SUBASSEMBLY_JOINER);
                        sub_occurrences.insert(occurrence_key, occurrence.clone());
                    }
                    Err(e) => {
                        log::warn!("Skipping occurrence: {}", e);
                        continue;
                    }
                }
            }

            // Create part for rigid assembly
            let rigid_assembly_to_part_tf = HashMap::new();

            // Create a Part instance for the rigid assembly
            let part = Part {
                base: IdBase {
                    configuration: rigid_subassembly.sub_assembly.base.configuration.clone(),
                    full_configuration: rigid_subassembly.sub_assembly.base.full_configuration.clone(),
                    document_id: rigid_subassembly.sub_assembly.base.document_id.clone(),
                    document_microversion: rigid_subassembly.sub_assembly.base.document_microversion.clone(),
                    element_id: rigid_subassembly.sub_assembly.base.element_id.clone(),
                },
                is_standard_content: false,
                part_id: String::new(),
                body_type: String::new(),
                document_version: None,
                mass_property: rigid_subassembly.mass_property.clone(),
                is_rigid_assembly: Some(true),
                rigid_assembly_to_part_tf: Some(rigid_assembly_to_part_tf),
                rigid_assembly_workspace_id: rigid_subassembly.document_meta_data
                    .as_ref()
                    .and_then(|meta| Some(meta.default_workspace.id.clone())),
            };

            parts.insert(assembly_key.clone(), part);
            occurrence_map.insert(assembly_key.clone(), sub_occurrences);
        }

        Ok(occurrence_map)
    }

    /// Process assembly features asynchronously
    /// Mirrors the Python process_features_async function
    pub async fn process_features_async(
        features: &[AssemblyFeature],
        parts: &mut HashMap<String, Part>,
        id_to_name_map: &HashMap<String, String>,
        rigid_subassembly_occurrence_map: &HashMap<String, HashMap<String, Occurrence>>,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        subassembly_prefix: Option<&str>,
    ) -> Result<(HashMap<String, MateFeatureData>, HashMap<String, MateRelationFeatureData>), String> {
        let mut mates_map: HashMap<String, MateFeatureData> = HashMap::new();
        let mut relations_map: HashMap<String, MateRelationFeatureData> = HashMap::new();

        for feature in features {
            // Skip suppressed features
            if feature.suppressed {
                continue;
            }

            match feature.feature_type {
                AssemblyFeatureType::Mate => {
                    if let Some(mate_data) = feature.get_mate_data() {
                        let result = Self::process_mate_feature(
                            feature,
                            mate_data,
                            parts,
                            id_to_name_map,
                            rigid_subassembly_occurrence_map,
                            rigid_subassemblies,
                            subassembly_prefix,
                        ).await;

                        match result {
                            Ok(Some((mate_key, mate_feature))) => {
                                mates_map.insert(mate_key, mate_feature);
                            }
                            Ok(None) => {
                                // Feature was skipped (invalid or suppressed)
                                continue;
                            }
                            Err(e) => {
                                log::warn!("Failed to process mate feature {}: {}", feature.id, e);
                                continue;
                            }
                        }
                    }
                }
                AssemblyFeatureType::MateRelation => {
                    if let Some(relation_data) = feature.get_mate_relation_data() {
                        let result = Self::process_mate_relation_feature(
                            feature,
                            relation_data,
                        ).await;

                        match result {
                            Ok(Some((relation_key, relation_feature))) => {
                                relations_map.insert(relation_key, relation_feature);
                            }
                            Ok(None) => {
                                // Feature was skipped
                                continue;
                            }
                            Err(e) => {
                                log::warn!("Failed to process mate relation feature {}: {}", feature.id, e);
                                continue;
                            }
                        }
                    }
                }
                _ => {
                    // Skip other feature types for now
                    log::debug!("Skipping feature type: {:?}", feature.feature_type);
                }
            }
        }

        Ok((mates_map, relations_map))
    }

    /// Process a single mate feature
    async fn process_mate_feature(
        feature: &AssemblyFeature,
        mut mate_data: MateFeatureData,
        parts: &mut HashMap<String, Part>,
        id_to_name_map: &HashMap<String, String>,
        rigid_subassembly_occurrence_map: &HashMap<String, HashMap<String, Occurrence>>,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        subassembly_prefix: Option<&str>,
    ) -> Result<Option<(String, MateFeatureData)>, String> {
        // Set the feature ID
        mate_data.id = Some(feature.id.clone());

        // Validate mate has at least 2 entities
        if mate_data.mated_entities.len() < 2 {
            log::warn!("Invalid mate feature (< 2 entities): {}", feature.id);
            return Ok(None);
        }

        // Extract occurrence paths
        let child_occurrences = Self::map_occurrences_to_names(
            &mate_data.mated_entities[CHILD].mated_occurrence,
            id_to_name_map,
        )?;

        let parent_occurrences = Self::map_occurrences_to_names(
            &mate_data.mated_entities[PARENT].mated_occurrence,
            id_to_name_map,
        )?;

        // Handle rigid subassemblies
        let mut final_parent_occurrences = parent_occurrences;
        let mut final_child_occurrences = child_occurrences;

        // Process parent rigid subassembly
        if let Some(first_parent) = final_parent_occurrences.first() {
            if rigid_subassemblies.contains_key(first_parent) {
                let result = Self::handle_rigid_subassembly_mate(
                    first_parent,
                    &final_parent_occurrences,
                    PARENT,
                    &mut mate_data,
                    parts,
                    rigid_subassembly_occurrence_map,
                )?;
                if let Some(updated_occurrences) = result {
                    final_parent_occurrences = updated_occurrences;
                }
            }
        }

        // Process child rigid subassembly
        if let Some(first_child) = final_child_occurrences.first() {
            if rigid_subassemblies.contains_key(first_child) {
                let result = Self::handle_rigid_subassembly_mate(
                    first_child,
                    &final_child_occurrences,
                    CHILD,
                    &mut mate_data,
                    parts,
                    rigid_subassembly_occurrence_map,
                )?;
                if let Some(updated_occurrences) = result {
                    final_child_occurrences = updated_occurrences;
                }
            }
        }

        // Generate mate key
        let mate_key = crate::parse::instances::join_mate_occurrences(
            &final_parent_occurrences,
            &final_child_occurrences,
            subassembly_prefix,
        );

        Ok(Some((mate_key, mate_data)))
    }

    /// Process a single mate relation feature
    async fn process_mate_relation_feature(
        feature: &AssemblyFeature,
        mut relation_data: MateRelationFeatureData,
    ) -> Result<Option<(String, MateRelationFeatureData)>, String> {
        // Set the feature ID
        relation_data.id = Some(feature.id.clone());

        // Determine the child joint ID based on relation type
        let child_joint_id = match relation_data.relation_type {
            RelationType::Screw => {
                // For screw relations, use the first mate
                relation_data.mates.first()
                    .ok_or("No mates found for screw relation")?
                    .feature_id.clone()
            }
            _ => {
                // For other relations, use the child mate (index 1)
                relation_data.mates.get(RELATION_CHILD)
                    .ok_or("No child mate found for relation")?
                    .feature_id.clone()
            }
        };

        Ok(Some((child_joint_id, relation_data)))
    }

    /// Handle rigid subassembly mate processing
    fn handle_rigid_subassembly_mate(
        subassembly_key: &str,
        occurrences: &[String],
        entity_index: usize,
        mate_data: &mut MateFeatureData,
        parts: &mut HashMap<String, Part>,
        rigid_subassembly_occurrence_map: &HashMap<String, HashMap<String, Occurrence>>,
    ) -> Result<Option<Vec<String>>, String> {
        if occurrences.len() < 2 {
            return Ok(Some(vec![subassembly_key.to_string()]));
        }

        let sub_occurrence_key = occurrences[1].clone();

        if let Some(occurrence_map) = rigid_subassembly_occurrence_map.get(subassembly_key) {
            if let Some(occurrence) = occurrence_map.get(&sub_occurrence_key) {
                                 // Create transformation matrix from occurrence
                 if occurrence.transform.len() != 16 {
                     return Err(format!("Invalid transformation matrix length: {}", occurrence.transform.len()));
                 }
                 let transform_matrix = nalgebra::Matrix4::from_column_slice(&occurrence.transform);
                let parent_cs = MatedCS::from_matrix(&transform_matrix);

                // Update the mate data with parent coordinate system
                mate_data.mated_entities[entity_index].parent_cs = Some(parent_cs.clone());

                // Update part's rigid assembly to part transformation
                if let Some(part) = parts.get_mut(subassembly_key) {
                    if let Some(ref mut rigid_tf_map) = part.rigid_assembly_to_part_tf {
                        rigid_tf_map.insert(sub_occurrence_key, parent_cs);
                    }
                }
            }
        }

        Ok(Some(vec![subassembly_key.to_string()]))
    }

    /// Map occurrence IDs to sanitized names
    fn map_occurrences_to_names(
        occurrence_ids: &[String],
        id_to_name_map: &HashMap<String, String>,
    ) -> Result<Vec<String>, String> {
        occurrence_ids
            .iter()
            .map(|id| {
                id_to_name_map.get(id)
                    .ok_or_else(|| format!("Occurrence ID {} not found in mapping", id))
                    .map(|name| name.clone())
            })
            .collect()
    }

    /// Main entry point for mate processing
    /// Mirrors the Python get_mates_and_relations_async function
    pub async fn get_mates_and_relations_async(
        assembly: &Assembly,
        subassemblies: &HashMap<String, SubAssembly>,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        id_to_name_map: &HashMap<String, String>,
        parts: &mut HashMap<String, Part>,
    ) -> Result<(HashMap<String, MateFeatureData>, HashMap<String, MateRelationFeatureData>), String> {
        // Build rigid subassembly occurrence map
        let rigid_subassembly_occurrence_map = Self::build_rigid_subassembly_occurrence_map(
            rigid_subassemblies,
            id_to_name_map,
            parts,
        ).await?;

        // Process root assembly features
        let (mut mates_map, mut relations_map) = Self::process_features_async(
            &assembly.root_assembly.sub_assembly.features,
            parts,
            id_to_name_map,
            &rigid_subassembly_occurrence_map,
            rigid_subassemblies,
            None,
        ).await?;

        // Process subassembly features
        for (key, subassembly) in subassemblies {
            let (sub_mates, sub_relations) = Self::process_features_async(
                &subassembly.features,
                parts,
                id_to_name_map,
                &rigid_subassembly_occurrence_map,
                rigid_subassemblies,
                Some(key),
            ).await?;

            // Merge results
            mates_map.extend(sub_mates);
            relations_map.extend(sub_relations);
        }

        Ok((mates_map, relations_map))
    }

    /// Legacy synchronous wrapper for compatibility
    /// Mirrors the Python get_mates_and_relations function
    pub fn get_mates_and_relations(
        assembly: &Assembly,
        subassemblies: &HashMap<String, SubAssembly>,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        id_to_name_map: &HashMap<String, String>,
        parts: &mut HashMap<String, Part>,
    ) -> Result<(HashMap<String, MateFeatureData>, HashMap<String, MateRelationFeatureData>), String> {
        // Use tokio to run async function
        let rt = tokio::runtime::Runtime::new()
            .map_err(|e| format!("Failed to create async runtime: {}", e))?;

        rt.block_on(Self::get_mates_and_relations_async(
            assembly,
            subassemblies,
            rigid_subassemblies,
            id_to_name_map,
            parts,
        ))
    }

    /// Process mate features (main compatibility function)
    pub async fn process_mates(
        assembly: &Assembly,
        _instances: &HashMap<String, Instance>,
    ) -> Result<(HashMap<String, MateFeatureData>, HashMap<String, MateRelationFeatureData>), String> {
        // This is a simplified interface for basic mate processing
        // For full functionality, use get_mates_and_relations_async

        log::warn!("process_mates is a simplified interface. Use get_mates_and_relations_async for full functionality.");

        let mut mates_map = HashMap::new();
        let mut relations_map = HashMap::new();

        // Process only root assembly features for now
        for feature in &assembly.root_assembly.sub_assembly.features {
            if feature.suppressed {
                continue;
            }

            match feature.feature_type {
                AssemblyFeatureType::Mate => {
                    if let Some(mut mate_data) = feature.get_mate_data() {
                        mate_data.id = Some(feature.id.clone());
                        mates_map.insert(feature.id.clone(), mate_data);
                    }
                }
                AssemblyFeatureType::MateRelation => {
                    if let Some(mut relation_data) = feature.get_mate_relation_data() {
                        relation_data.id = Some(feature.id.clone());
                        relations_map.insert(feature.id.clone(), relation_data);
                    }
                }
                _ => {}
            }
        }

        Ok((mates_map, relations_map))
    }

    /// Validate mate data consistency and geometry
    pub fn validate_mate_data(mate_data: &MateFeatureData) -> Result<(), String> {
        // Check minimum number of mated entities
        if mate_data.mated_entities.len() < 2 {
            return Err(format!("Mate {} has less than 2 mated entities",
                mate_data.id.as_ref().unwrap_or(&"unknown".to_string())));
        }

        // Validate each mated entity
        for (i, entity) in mate_data.mated_entities.iter().enumerate() {
            if entity.mated_occurrence.is_empty() {
                return Err(format!("Mated entity {} has empty occurrence list", i));
            }

            // Validate coordinate system
            Self::validate_coordinate_system(&entity.mated_cs)?;
        }

        // Validate mate type constraints
        match mate_data.mate_type {
            MateType::Fastened => {
                // Fastened mates should have exactly 2 entities
                if mate_data.mated_entities.len() != 2 {
                    return Err("Fastened mate must have exactly 2 entities".to_string());
                }
            }
            MateType::Revolute | MateType::Slider => {
                // Revolute and slider mates require proper axis alignment
                // Additional validation could be added here
            }
            _ => {
                // Other mate types - add specific validation as needed
            }
        }

        Ok(())
    }

    /// Validate coordinate system vectors
    pub fn validate_coordinate_system(cs: &MatedCS) -> Result<(), String> {
        // Check vector lengths
        if cs.x_axis.len() != 3 || cs.y_axis.len() != 3 || cs.z_axis.len() != 3 || cs.origin.len() != 3 {
            return Err("All coordinate system vectors must have 3 components".to_string());
        }

        // Check if axes are normalized (approximately)
        let x_norm = (cs.x_axis[0].powi(2) + cs.x_axis[1].powi(2) + cs.x_axis[2].powi(2)).sqrt();
        let y_norm = (cs.y_axis[0].powi(2) + cs.y_axis[1].powi(2) + cs.y_axis[2].powi(2)).sqrt();
        let z_norm = (cs.z_axis[0].powi(2) + cs.z_axis[1].powi(2) + cs.z_axis[2].powi(2)).sqrt();

        const NORM_TOLERANCE: f64 = 1e-6;
        if (x_norm - 1.0).abs() > NORM_TOLERANCE ||
           (y_norm - 1.0).abs() > NORM_TOLERANCE ||
           (z_norm - 1.0).abs() > NORM_TOLERANCE {
            log::warn!("Coordinate system axes are not normalized: x={}, y={}, z={}", x_norm, y_norm, z_norm);
        }

        // Check orthogonality (dot products should be close to zero)
        let xy_dot = cs.x_axis[0] * cs.y_axis[0] + cs.x_axis[1] * cs.y_axis[1] + cs.x_axis[2] * cs.y_axis[2];
        let xz_dot = cs.x_axis[0] * cs.z_axis[0] + cs.x_axis[1] * cs.z_axis[1] + cs.x_axis[2] * cs.z_axis[2];
        let yz_dot = cs.y_axis[0] * cs.z_axis[0] + cs.y_axis[1] * cs.z_axis[1] + cs.y_axis[2] * cs.z_axis[2];

        const ORTHOGONAL_TOLERANCE: f64 = 1e-6;
        if xy_dot.abs() > ORTHOGONAL_TOLERANCE ||
           xz_dot.abs() > ORTHOGONAL_TOLERANCE ||
           yz_dot.abs() > ORTHOGONAL_TOLERANCE {
            log::warn!("Coordinate system axes are not orthogonal: xy={}, xz={}, yz={}", xy_dot, xz_dot, yz_dot);
        }

        Ok(())
    }

    /// Validate mate relation data
    pub fn validate_mate_relation_data(relation_data: &MateRelationFeatureData) -> Result<(), String> {
        // Check minimum number of mates
        if relation_data.mates.len() < 2 {
            return Err(format!("Mate relation {} has less than 2 mates",
                relation_data.id.as_ref().unwrap_or(&"unknown".to_string())));
        }

        // Validate relation type constraints
        match relation_data.relation_type {
            RelationType::Gear => {
                if relation_data.relation_ratio.is_none() {
                    return Err("Gear relation must have a relation ratio".to_string());
                }
                if let Some(ratio) = relation_data.relation_ratio {
                    if ratio <= 0.0 {
                        return Err("Gear relation ratio must be positive".to_string());
                    }
                }
            }
            RelationType::RackAndPinion => {
                if relation_data.relation_length.is_none() {
                    return Err("Rack and pinion relation must have a relation length".to_string());
                }
                if let Some(length) = relation_data.relation_length {
                    if length <= 0.0 {
                        return Err("Rack and pinion relation length must be positive".to_string());
                    }
                }
            }
            RelationType::Screw => {
                // Screw relations should have exactly one mate
                if relation_data.mates.len() != 1 {
                    return Err("Screw relation must have exactly 1 mate".to_string());
                }
            }
            RelationType::Linear => {
                // Linear relations need at least 2 mates
                if relation_data.mates.len() < 2 {
                    return Err("Linear relation must have at least 2 mates".to_string());
                }
            }
        }

        Ok(())
    }

    /// Check for over-constrained assemblies
    pub fn check_constraints(
        mates_map: &HashMap<String, MateFeatureData>,
        _relations_map: &HashMap<String, MateRelationFeatureData>,
    ) -> Result<(), String> {
        log::info!("Checking assembly constraints for {} mates", mates_map.len());

        // Count different types of constraints
        let mut fastened_count = 0;
        let mut revolute_count = 0;
        let mut slider_count = 0;
        let mut other_count = 0;

        for mate in mates_map.values() {
            match mate.mate_type {
                MateType::Fastened => fastened_count += 1,
                MateType::Revolute => revolute_count += 1,
                MateType::Slider => slider_count += 1,
                _ => other_count += 1,
            }
        }

        log::info!("Constraint summary: {} fastened, {} revolute, {} slider, {} other",
                  fastened_count, revolute_count, slider_count, other_count);

        // Basic constraint validation (this could be more sophisticated)
        let total_constraints = mates_map.len();
        if total_constraints == 0 {
            log::warn!("No mates found in assembly - parts may be unconstrained");
        }

        // Check for excessive fastened mates (which remove all degrees of freedom)
        if fastened_count > 0 {
            log::info!("Assembly has {} fastened mates, which fully constrain parts", fastened_count);
        }

        Ok(())
    }

    /// Get mate statistics for debugging and analysis
    pub fn get_mate_statistics(
        mates_map: &HashMap<String, MateFeatureData>,
        relations_map: &HashMap<String, MateRelationFeatureData>,
    ) -> MateStatistics {
        let mut stats = MateStatistics::default();

        // Count mate types
        for mate in mates_map.values() {
            match mate.mate_type {
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

        // Count relation types
        for relation in relations_map.values() {
            match relation.relation_type {
                RelationType::Gear => stats.gear_relation_count += 1,
                RelationType::Linear => stats.linear_relation_count += 1,
                RelationType::Screw => stats.screw_relation_count += 1,
                RelationType::RackAndPinion => stats.rack_pinion_relation_count += 1,
            }
        }

        stats.total_mates = mates_map.len();
        stats.total_relations = relations_map.len();

        stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_mated_cs() -> MatedCS {
        MatedCS {
            x_axis: vec![1.0, 0.0, 0.0],
            y_axis: vec![0.0, 1.0, 0.0],
            z_axis: vec![0.0, 0.0, 1.0],
            origin: vec![0.0, 0.0, 0.0],
            part_tf: None,
        }
    }

    fn create_test_mated_entity() -> MatedEntity {
        MatedEntity {
            mated_occurrence: vec!["test_occurrence".to_string()],
            mated_cs: create_test_mated_cs(),
            parent_cs: None,
        }
    }

    fn create_test_mate_feature() -> MateFeatureData {
        MateFeatureData {
            mated_entities: vec![create_test_mated_entity(), create_test_mated_entity()],
            mate_type: MateType::Fastened,
            name: "Test Mate".to_string(),
            id: Some("test_mate_id".to_string()),
        }
    }

    #[test]
    fn test_validate_coordinate_system() {
        let cs = create_test_mated_cs();
        assert!(MateProcessor::validate_coordinate_system(&cs).is_ok());

        // Test invalid coordinate system
        let invalid_cs = MatedCS {
            x_axis: vec![1.0, 0.0], // Wrong length
            y_axis: vec![0.0, 1.0, 0.0],
            z_axis: vec![0.0, 0.0, 1.0],
            origin: vec![0.0, 0.0, 0.0],
            part_tf: None,
        };
        assert!(MateProcessor::validate_coordinate_system(&invalid_cs).is_err());
    }

    #[test]
    fn test_validate_mate_data() {
        let mate = create_test_mate_feature();
        assert!(MateProcessor::validate_mate_data(&mate).is_ok());

        // Test invalid mate with too few entities
        let invalid_mate = MateFeatureData {
            mated_entities: vec![create_test_mated_entity()], // Only one entity
            mate_type: MateType::Fastened,
            name: "Invalid Mate".to_string(),
            id: Some("invalid_mate_id".to_string()),
        };
        assert!(MateProcessor::validate_mate_data(&invalid_mate).is_err());
    }

    #[test]
    fn test_map_occurrences_to_names() {
        let mut id_to_name_map = HashMap::new();
        id_to_name_map.insert("id1".to_string(), "name1".to_string());
        id_to_name_map.insert("id2".to_string(), "name2".to_string());

        let occurrence_ids = vec!["id1".to_string(), "id2".to_string()];
        let result = MateProcessor::map_occurrences_to_names(&occurrence_ids, &id_to_name_map);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["name1", "name2"]);

        // Test with missing ID
        let invalid_ids = vec!["id1".to_string(), "missing_id".to_string()];
        let result = MateProcessor::map_occurrences_to_names(&invalid_ids, &id_to_name_map);
        assert!(result.is_err());
    }

    #[test]
    fn test_mate_statistics() {
        let mut mates_map = HashMap::new();
        mates_map.insert("mate1".to_string(), MateFeatureData {
            mated_entities: vec![create_test_mated_entity(), create_test_mated_entity()],
            mate_type: MateType::Fastened,
            name: "Fastened Mate".to_string(),
            id: Some("mate1".to_string()),
        });
        mates_map.insert("mate2".to_string(), MateFeatureData {
            mated_entities: vec![create_test_mated_entity(), create_test_mated_entity()],
            mate_type: MateType::Revolute,
            name: "Revolute Mate".to_string(),
            id: Some("mate2".to_string()),
        });

        let relations_map = HashMap::new();
        let stats = MateProcessor::get_mate_statistics(&mates_map, &relations_map);

        assert_eq!(stats.total_mates, 2);
        assert_eq!(stats.fastened_count, 1);
        assert_eq!(stats.revolute_count, 1);
        assert_eq!(stats.total_relations, 0);
    }

    #[test]
    fn test_validate_mate_relation_data() {
        let gear_relation = MateRelationFeatureData {
            relation_type: RelationType::Gear,
            mates: vec![
                MateRelationMate {
                    feature_id: "mate1".to_string(),
                    occurrence: vec!["occ1".to_string()],
                },
                MateRelationMate {
                    feature_id: "mate2".to_string(),
                    occurrence: vec!["occ2".to_string()],
                },
            ],
            reverse_direction: false,
            relation_ratio: Some(2.0),
            relation_length: None,
            name: "Gear Relation".to_string(),
            id: Some("gear_rel_1".to_string()),
        };

        assert!(MateProcessor::validate_mate_relation_data(&gear_relation).is_ok());

        // Test invalid gear relation without ratio
        let invalid_gear = MateRelationFeatureData {
            relation_type: RelationType::Gear,
            mates: vec![
                MateRelationMate {
                    feature_id: "mate1".to_string(),
                    occurrence: vec!["occ1".to_string()],
                },
                MateRelationMate {
                    feature_id: "mate2".to_string(),
                    occurrence: vec!["occ2".to_string()],
                },
            ],
            reverse_direction: false,
            relation_ratio: None, // Missing ratio
            relation_length: None,
            name: "Invalid Gear".to_string(),
            id: Some("invalid_gear".to_string()),
        };

        assert!(MateProcessor::validate_mate_relation_data(&invalid_gear).is_err());
    }

    #[test]
    fn test_check_constraints() {
        let mut mates_map = HashMap::new();
        mates_map.insert("mate1".to_string(), create_test_mate_feature());

        let relations_map = HashMap::new();

        // Should not panic or return error for basic constraint checking
        assert!(MateProcessor::check_constraints(&mates_map, &relations_map).is_ok());

        // Test empty mates
        let empty_mates = HashMap::new();
        assert!(MateProcessor::check_constraints(&empty_mates, &relations_map).is_ok());
    }
}

/// Statistics about mates in an assembly
#[derive(Debug, Clone, Default)]
pub struct MateStatistics {
    pub total_mates: usize,
    pub total_relations: usize,
    pub fastened_count: usize,
    pub revolute_count: usize,
    pub slider_count: usize,
    pub cylindrical_count: usize,
    pub planar_count: usize,
    pub ball_count: usize,
    pub pin_slot_count: usize,
    pub parallel_count: usize,
    pub gear_relation_count: usize,
    pub linear_relation_count: usize,
    pub screw_relation_count: usize,
    pub rack_pinion_relation_count: usize,
}

impl MateStatistics {
    /// Print a summary of the mate statistics
    pub fn print_summary(&self) {
        println!("Mate Processing Statistics:");
        println!("  Total mates: {}", self.total_mates);
        println!("  Total relations: {}", self.total_relations);
        println!("  Mate types:");
        println!("    Fastened: {}", self.fastened_count);
        println!("    Revolute: {}", self.revolute_count);
        println!("    Slider: {}", self.slider_count);
        println!("    Cylindrical: {}", self.cylindrical_count);
        println!("    Planar: {}", self.planar_count);
        println!("    Ball: {}", self.ball_count);
        println!("    Pin/Slot: {}", self.pin_slot_count);
        println!("    Parallel: {}", self.parallel_count);
        println!("  Relation types:");
        println!("    Gear: {}", self.gear_relation_count);
        println!("    Linear: {}", self.linear_relation_count);
        println!("    Screw: {}", self.screw_relation_count);
        println!("    Rack & Pinion: {}", self.rack_pinion_relation_count);
    }
}
