use std::collections::HashMap;
use futures::future::join_all;
use crate::parse::models::*;
use crate::parse::{SUBASSEMBLY_JOINER};
use crate::client::OnshapeClient;
// use crate::model::DocumentMetaData; // Currently unused

/// Utilities for sanitizing names (mirrors Python get_sanitized_name)
pub fn sanitize_name(name: &str) -> String {
    name.chars()
        .map(|c| match c {
            ' ' | '-' | '(' | ')' | '[' | ']' | '{' | '}' | '<' | '>' | '.' | ',' | '/' | '\\' | ':' | ';' | '?' | '"' | '\'' | '|' | '!' | '@' | '#' | '$' | '%' | '^' | '&' | '*' | '+' | '=' => '_',
            _ => c,
        })
        .collect::<String>()
        .trim_matches('_')
        .to_string()
}

/// Result type for instance traversal
pub type InstanceMap = HashMap<String, Instance>;
pub type OccurrenceMap = HashMap<String, Occurrence>;
pub type IdToNameMap = HashMap<String, String>;

/// Assembly traversal results
#[derive(Debug, Clone)]
pub struct TraversalResult {
    pub instance_map: InstanceMap,
    pub occurrence_map: OccurrenceMap,
    pub id_to_name_map: IdToNameMap,
}

/// Instance traversal engine
pub struct InstanceTraverser {
    max_depth: usize,
    current_depth: usize,
}

impl InstanceTraverser {
    pub fn new(max_depth: usize) -> Self {
        Self {
            max_depth,
            current_depth: 0,
        }
    }

    /// Traverse assembly instances asynchronously
    /// Mirrors the Python traverse_instances_async function
    pub async fn traverse_instances_async(
        &mut self,
        root: &RootAssembly,
        prefix: &str,
        assembly: &Assembly,
        instance_map: &mut InstanceMap,
        id_to_name_map: &mut IdToNameMap,
    ) -> Result<(), String> {
        self.traverse_subassembly_async(
            &root.sub_assembly,
            prefix,
            assembly,
            instance_map,
            id_to_name_map,
        ).await
    }

    /// Traverse subassembly instances
    fn traverse_subassembly_async<'a>(
        &'a mut self,
        root: &'a SubAssembly,
        prefix: &'a str,
        assembly: &'a Assembly,
        instance_map: &'a mut InstanceMap,
        id_to_name_map: &'a mut IdToNameMap,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), String>> + 'a>> {
        Box::pin(async move {
        let is_rigid = self.current_depth >= self.max_depth;

        if is_rigid {
            log::debug!(
                "Max depth {} reached. Assuming all sub-assemblies to be rigid at depth {}.",
                self.max_depth, self.current_depth
            );
        }

        // Process all instances in this subassembly
        for instance in &root.instances {
            let sanitized_name = sanitize_name(instance.get_name());
            log::debug!("Parsing instance: {}", sanitized_name);

            let instance_id = if prefix.is_empty() {
                sanitized_name.clone()
            } else {
                format!("{}{}{}", prefix, SUBASSEMBLY_JOINER, sanitized_name)
            };

            id_to_name_map.insert(instance.get_id().to_string(), sanitized_name);

            // Mark assembly instances as rigid if needed
            let mut instance_clone = instance.clone();
            if let Instance::Assembly(ref mut assembly_instance) = instance_clone {
                assembly_instance.is_rigid = Some(is_rigid);
            }

            instance_map.insert(instance_id.clone(), instance_clone);

            // Recursively process subassemblies if not at max depth
            if matches!(instance.get_type(), InstanceType::Assembly) && !is_rigid {
                self.current_depth += 1;

                // Find matching subassembly and traverse it
                for sub_assembly in &assembly.sub_assemblies {
                    if sub_assembly.uid() == instance.uid() {
                        self.traverse_subassembly_async(
                            sub_assembly,
                            &instance_id,
                            assembly,
                            instance_map,
                            id_to_name_map,
                        ).await?;
                        break;
                    }
                }

                self.current_depth -= 1;
            }
        }

        Ok(())
        })
    }

    /// Get instances synchronously (main entry point)
    /// Mirrors the Python get_instances function
    pub async fn get_instances(
        assembly: &Assembly,
        max_depth: usize,
    ) -> Result<TraversalResult, String> {
        let mut instance_map = HashMap::new();
        let mut id_to_name_map = HashMap::new();

        let mut traverser = Self::new(max_depth);
        traverser.traverse_instances_async(
            &assembly.root_assembly,
            "",
            assembly,
            &mut instance_map,
            &mut id_to_name_map,
        ).await?;

        // Get occurrences
        let occurrence_map = Self::get_occurrences(assembly, &id_to_name_map, max_depth);

        Ok(TraversalResult {
            instance_map,
            occurrence_map,
            id_to_name_map,
        })
    }

    /// Get occurrences from assembly
    /// Mirrors the Python get_occurrences function
    fn get_occurrences(
        assembly: &Assembly,
        id_to_name_map: &IdToNameMap,
        max_depth: usize,
    ) -> OccurrenceMap {
        assembly.root_assembly.occurrences
            .iter()
            .filter_map(|occurrence| {
                // Filter by max depth
                if occurrence.path.len() > max_depth + 1 {
                    return None;
                }

                // Build occurrence path from ID mapping
                let path_names: Vec<String> = occurrence.path
                    .iter()
                    .filter_map(|path_id| id_to_name_map.get(path_id).cloned())
                    .collect();

                if path_names.is_empty() {
                    return None;
                }

                let occurrence_key = path_names.join(SUBASSEMBLY_JOINER);
                Some((occurrence_key, occurrence.clone()))
            })
            .collect()
    }
}

/// Subassembly fetcher for rigid and flexible subassemblies
pub struct SubassemblyFetcher;

impl SubassemblyFetcher {
    /// Get subassemblies with concurrent fetching
    /// Mirrors the Python get_subassemblies_async function
    pub async fn get_subassemblies(
        assembly: &Assembly,
        client: &OnshapeClient,
        instances: &InstanceMap,
    ) -> Result<(HashMap<String, SubAssembly>, HashMap<String, RootAssembly>), String> {
        let mut subassembly_map = HashMap::new();
        let mut rigid_subassembly_map = HashMap::new();

        // Group instances by UID for efficient processing
        let mut subassembly_instances: HashMap<String, Vec<String>> = HashMap::new();
        let mut rigid_subassembly_instances: HashMap<String, Vec<String>> = HashMap::new();

        for (instance_key, instance) in instances {
            if let Instance::Assembly(assembly_instance) = instance {
                let uid = assembly_instance.uid();

                if assembly_instance.is_rigid.unwrap_or(false) {
                    rigid_subassembly_instances
                        .entry(uid)
                        .or_default()
                        .push(instance_key.clone());
                } else {
                    subassembly_instances
                        .entry(uid)
                        .or_default()
                        .push(instance_key.clone());
                }
            }
        }

        // Process flexible subassemblies
        for subassembly in &assembly.sub_assemblies {
            let uid = subassembly.uid();

            if let Some(instance_keys) = subassembly_instances.get(&uid) {
                // Check if subassembly is actually rigid (no features or only mate groups)
                let is_rigid = subassembly.features.is_empty() ||
                    subassembly.features.iter().all(|f|
                        matches!(f.feature_type, AssemblyFeatureType::MateGroup)
                    );

                if is_rigid {
                    // Fetch as rigid subassemblies concurrently
                    let fetch_tasks: Vec<_> = instance_keys.iter().map(|key| {
                        Self::fetch_rigid_subassembly(subassembly, key.clone(), client)
                    }).collect();

                    let results = join_all(fetch_tasks).await;
                    for (key, result) in instance_keys.iter().zip(results) {
                        match result {
                            Ok(root_assembly) => {
                                rigid_subassembly_map.insert(key.clone(), root_assembly);
                            }
                            Err(e) => {
                                log::error!("Failed to fetch rigid subassembly for {}: {}", key, e);
                            }
                        }
                    }
                } else {
                    // Add as flexible subassembly
                    for key in instance_keys {
                        subassembly_map.insert(key.clone(), subassembly.clone());
                    }
                }
            }
        }

        // Process rigid subassembly instances
        for subassembly in &assembly.sub_assemblies {
            let uid = subassembly.uid();

            if let Some(instance_keys) = rigid_subassembly_instances.get(&uid) {
                let fetch_tasks: Vec<_> = instance_keys.iter().map(|key| {
                    Self::fetch_rigid_subassembly(subassembly, key.clone(), client)
                }).collect();

                let results = join_all(fetch_tasks).await;
                for (key, result) in instance_keys.iter().zip(results) {
                    match result {
                        Ok(root_assembly) => {
                            rigid_subassembly_map.insert(key.clone(), root_assembly);
                        }
                        Err(e) => {
                            log::error!("Failed to fetch rigid subassembly for {}: {}", key, e);
                        }
                    }
                }
            }
        }

        Ok((subassembly_map, rigid_subassembly_map))
    }

    /// Fetch a rigid subassembly
    /// Mirrors the Python fetch_rigid_subassemblies_async function
    async fn fetch_rigid_subassembly(
        subassembly: &SubAssembly,
        _key: String,
        _client: &OnshapeClient,
    ) -> Result<RootAssembly, String> {
        // This would use the client to fetch the rigid subassembly
        // For now, we'll create a placeholder implementation

        // In the actual implementation, this would call:
        // client.get_root_assembly(
        //     &subassembly.base.document_id,
        //     "m", // workspace type for microversion
        //     &subassembly.base.document_microversion,
        //     &subassembly.base.element_id,
        //     Some(true), // with_mass_properties
        //     Some(false), // log_response
        // ).await

        // For now, return a minimal RootAssembly
        Ok(RootAssembly {
            sub_assembly: SubAssembly {
                base: subassembly.base.clone(),
                instances: vec![],
                patterns: vec![],
                features: vec![],
            },
            occurrences: vec![],
            mass_property: None,
            document_meta_data: None,
        })
    }
}

/// Part fetcher with concurrent mass property loading
pub struct PartFetcher;

impl PartFetcher {
    /// Get parts with concurrent mass property fetching
    /// Mirrors the Python get_parts function
    pub async fn get_parts(
        assembly: &Assembly,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        client: &OnshapeClient,
        instances: &InstanceMap,
    ) -> Result<HashMap<String, Part>, String> {
        let mut part_map = HashMap::new();

        // Group part instances by UID
        let mut part_instances: HashMap<String, Vec<String>> = HashMap::new();

        for (key, instance) in instances {
            if let Instance::Part(part_instance) = instance {
                part_instances
                    .entry(part_instance.uid())
                    .or_default()
                    .push(key.clone());
            }
        }

        // Process parts with concurrent mass property fetching
        let mut fetch_tasks = vec![];

        for part in &assembly.parts {
            if let Some(instance_keys) = part_instances.get(&part.uid()) {
                for key in instance_keys {
                    let part_clone = part.clone();
                    let key_clone = key.clone();
                    let client_clone = client.clone();
                    let rigid_subassemblies_clone = rigid_subassemblies.clone();

                    fetch_tasks.push(async move {
                        Self::fetch_part_with_mass_properties(
                            part_clone,
                            key_clone,
                            &client_clone,
                            &rigid_subassemblies_clone,
                        ).await
                    });
                }
            }
        }

        // Execute all fetch tasks concurrently
        let results = join_all(fetch_tasks).await;

        for result in results {
            match result {
                Ok((key, part)) => {
                    part_map.insert(key, part);
                }
                Err(e) => {
                    log::error!("Failed to fetch part: {}", e);
                }
            }
        }

        Ok(part_map)
    }

    /// Fetch a part with its mass properties
    /// Mirrors the Python _fetch_mass_properties_async function
    async fn fetch_part_with_mass_properties(
        part: Part,
        key: String,
        _client: &OnshapeClient,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
    ) -> Result<(String, Part), String> {
        // Check if this part is from a rigid subassembly
        let assembly_key = key.split(SUBASSEMBLY_JOINER).next().unwrap_or("");

        if !rigid_subassemblies.contains_key(assembly_key) {
            // Fetch mass properties for non-rigid assembly parts
            // This would use the client to fetch mass properties
            // For now, we'll keep the existing mass_property if any

            // In the actual implementation, this would call:
            // part.mass_property = Some(client.get_mass_property(
            //     &part.base.document_id,
            //     "m", // workspace type for microversion
            //     &part.base.document_microversion,
            //     &part.base.element_id,
            //     &part.part_id,
            // ).await?);

            log::info!("Fetching mass properties for part: {}, {}", part.uid(), part.part_id);
        }

        Ok((key, part))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sanitize_name() {
        assert_eq!(sanitize_name("Part 1 <2>"), "Part_1_2");
        assert_eq!(sanitize_name("Assembly-Name"), "Assembly_Name");
        assert_eq!(sanitize_name("Normal_Name"), "Normal_Name");
        assert_eq!(sanitize_name("   spaces   "), "spaces");
    }

    #[test]
    fn test_instance_traverser_creation() {
        let traverser = InstanceTraverser::new(5);
        assert_eq!(traverser.max_depth, 5);
        assert_eq!(traverser.current_depth, 0);
    }
}
