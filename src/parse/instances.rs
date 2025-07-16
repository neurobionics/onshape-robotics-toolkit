use std::collections::HashMap;
use futures::future::join_all;
use crate::parse::models::*;
use crate::parse::{SUBASSEMBLY_JOINER};
use crate::client::{OnshapeClient, AsyncOnshapeClient};
use crate::endpoint::AsyncQuery;
use crate::endpoints::{GetAssembly, GetPartMassProperties, GetAssemblyMassProperties};
use crate::model::{Assembly as OnshapeAssembly, MassProperties};

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

/// Configuration for assembly parsing
#[derive(Debug, Clone)]
pub struct ParseConfig {
    pub max_depth: usize,
    pub configuration: Option<String>,
    pub with_mass_properties: bool,
    pub include_mate_features: bool,
    pub include_mate_connectors: bool,
    pub include_non_solids: bool,
    pub log_response: bool,
}

impl Default for ParseConfig {
    fn default() -> Self {
        Self {
            max_depth: 0,
            configuration: None,
            with_mass_properties: true,
            include_mate_features: true,
            include_mate_connectors: true,
            include_non_solids: false,
            log_response: false,
        }
    }
}

/// Instance traversal engine
pub struct InstanceTraverser {
    config: ParseConfig,
    current_depth: usize,
}

impl InstanceTraverser {
    pub fn new(config: ParseConfig) -> Self {
        Self {
            config,
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
            let is_rigid = self.current_depth >= self.config.max_depth;

            if is_rigid {
                log::debug!(
                    "Max depth {} reached. Assuming all sub-assemblies to be rigid at depth {}.",
                    self.config.max_depth, self.current_depth
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
        config: ParseConfig,
    ) -> Result<TraversalResult, String> {
        let mut instance_map = HashMap::new();
        let mut id_to_name_map = HashMap::new();

        let mut traverser = Self::new(config.clone());
        traverser.traverse_instances_async(
            &assembly.root_assembly,
            "",
            assembly,
            &mut instance_map,
            &mut id_to_name_map,
        ).await?;

        // Get occurrences
        let occurrence_map = Self::get_occurrences(assembly, &id_to_name_map, config.max_depth);

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
    pub async fn get_subassemblies_async(
        assembly: &Assembly,
        client: &AsyncOnshapeClient,
        instances: &InstanceMap,
        config: &ParseConfig,
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
                        Self::fetch_rigid_subassembly_async(subassembly, key.clone(), client, config)
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
                    Self::fetch_rigid_subassembly_async(subassembly, key.clone(), client, config)
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

    /// Synchronous wrapper (to be implemented later)
    pub fn get_subassemblies(
        _assembly: &Assembly,
        _client: &OnshapeClient,
        _instances: &InstanceMap,
        _config: &ParseConfig,
    ) -> Result<(HashMap<String, SubAssembly>, HashMap<String, RootAssembly>), String> {
        // TODO: Implement synchronous wrapper once client credentials access is resolved
        Err("Synchronous version not yet implemented. Use get_subassemblies_async instead.".to_string())
    }

    /// Fetch a rigid subassembly
    /// Mirrors the Python fetch_rigid_subassemblies_async function
    async fn fetch_rigid_subassembly_async(
        subassembly: &SubAssembly,
        key: String,
        client: &AsyncOnshapeClient,
        config: &ParseConfig,
    ) -> Result<RootAssembly, String> {
        log::info!("Fetching rigid subassembly for key: {}", key);

        let endpoint = GetAssembly {
            did: &subassembly.base.document_id,
            wtype: "m", // microversion workspace type
            wid: &subassembly.base.document_microversion,
            eid: &subassembly.base.element_id,
            configuration: config.configuration.as_deref().unwrap_or("default"),
            include_mate_features: config.include_mate_features,
            include_mate_connectors: config.include_mate_connectors,
            include_non_solids: config.include_non_solids,
        };

        let assembly: OnshapeAssembly = AsyncQuery::query(&endpoint, client).await
            .map_err(|e| format!("Failed to fetch assembly: {}", e))?;

        let mut root_assembly: RootAssembly = assembly.root_assembly.into();

        // Fetch mass properties if requested
        if config.with_mass_properties {
            match Self::fetch_assembly_mass_properties(
                &subassembly.base.document_id,
                &subassembly.base.document_microversion,
                &subassembly.base.element_id,
                client
            ).await {
                Ok(mass_props) => {
                                     root_assembly.mass_property = Some(mass_props);
                }
                Err(e) => {
                    log::warn!("Failed to fetch mass properties for {}: {}", key, e);
                }
            }
        }

        Ok(root_assembly)
    }

    /// Fetch assembly mass properties
    async fn fetch_assembly_mass_properties(
        did: &str,
        wid: &str,
        eid: &str,
        client: &AsyncOnshapeClient,
    ) -> Result<MassProperties, String> {
        let endpoint = GetAssemblyMassProperties {
            did,
            wtype: "m",
            wid,
            eid,
        };

        AsyncQuery::query(&endpoint, client).await
            .map_err(|e| format!("Failed to fetch assembly mass properties: {}", e))
    }
}

/// Part fetcher with concurrent mass property loading
pub struct PartFetcher;

impl PartFetcher {
    /// Get parts with concurrent mass property fetching
    /// Mirrors the Python get_parts function
    pub async fn get_parts_async(
        assembly: &Assembly,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        client: &AsyncOnshapeClient,
        instances: &InstanceMap,
        config: &ParseConfig,
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
                    let config_clone = config.clone();

                    fetch_tasks.push(async move {
                        Self::fetch_part_with_mass_properties_async(
                            part_clone,
                            key_clone,
                            &client_clone,
                            &rigid_subassemblies_clone,
                            &config_clone,
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

    /// Synchronous wrapper (to be implemented later)
    pub fn get_parts(
        _assembly: &Assembly,
        _rigid_subassemblies: &HashMap<String, RootAssembly>,
        _client: &OnshapeClient,
        _instances: &InstanceMap,
        _config: &ParseConfig,
    ) -> Result<HashMap<String, Part>, String> {
        // TODO: Implement synchronous wrapper once client credentials access is resolved
        Err("Synchronous version not yet implemented. Use get_parts_async instead.".to_string())
    }

    /// Fetch a part with its mass properties
    /// Mirrors the Python _fetch_mass_properties_async function
    async fn fetch_part_with_mass_properties_async(
        mut part: Part,
        key: String,
        client: &AsyncOnshapeClient,
        rigid_subassemblies: &HashMap<String, RootAssembly>,
        config: &ParseConfig,
    ) -> Result<(String, Part), String> {
        // Check if this part is from a rigid subassembly
        let assembly_key = key.split(SUBASSEMBLY_JOINER).next().unwrap_or("");

        if !rigid_subassemblies.contains_key(assembly_key) && config.with_mass_properties {
            // Fetch mass properties for non-rigid assembly parts
            log::info!("Fetching mass properties for part: {}, {}", part.uid(), part.part_id);

            match Self::fetch_part_mass_properties(
                &part.base.document_id,
                &part.base.document_microversion,
                &part.base.element_id,
                &part.part_id,
                client,
            ).await {
                Ok(mass_props) => {
                    part.mass_property = Some(mass_props);
                }
                Err(e) => {
                    log::error!("Failed to fetch mass properties for part {}: {}", part.part_id, e);
                }
            }
        }

        Ok((key, part))
    }

    /// Fetch part mass properties
    async fn fetch_part_mass_properties(
        did: &str,
        wid: &str,
        eid: &str,
        part_id: &str,
        client: &AsyncOnshapeClient,
    ) -> Result<MassProperties, String> {
        let endpoint = GetPartMassProperties {
            did,
            wtype: "m",
            wid,
            eid,
            part_id,
            use_mass_properties_overrides: true,
        };

        AsyncQuery::query(&endpoint, client).await
            .map_err(|e| format!("Failed to fetch part mass properties: {}", e))
    }
}

/// Helper functions for occurrence management
pub fn get_occurrence_name(
    occurrences: &[String],
    subassembly_prefix: Option<&str>
) -> String {
    let prefix = if let Some(prefix) = subassembly_prefix {
        format!("{}{}", prefix, SUBASSEMBLY_JOINER)
    } else {
        String::new()
    };
    format!("{}{}", prefix, occurrences.join(SUBASSEMBLY_JOINER))
}

/// Join two occurrence paths with a mate joiner
/// Mirrors the Python join_mate_occurrences function
pub fn join_mate_occurrences(
    parent: &[String],
    child: &[String],
    prefix: Option<&str>
) -> String {
    let parent_occurrence = get_occurrence_name(parent, prefix);
    let child_occurrence = get_occurrence_name(child, prefix);
    format!("{}{}{}", parent_occurrence, crate::parse::MATE_JOINER, child_occurrence)
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
    fn test_parse_config_default() {
        let config = ParseConfig::default();
        assert_eq!(config.max_depth, 0);
        assert_eq!(config.configuration, None);
        assert!(config.with_mass_properties);
        assert!(config.include_mate_features);
        assert!(config.include_mate_connectors);
        assert!(!config.include_non_solids);
        assert!(!config.log_response);
    }

    #[test]
    fn test_get_occurrence_name() {
        assert_eq!(
            get_occurrence_name(&["part1".to_string()], Some("sub1")),
            "sub1_SUB_part1"
        );
        assert_eq!(
            get_occurrence_name(&["part1".to_string(), "part2".to_string()], None),
            "part1_SUB_part2"
        );
    }

    #[test]
    fn test_join_mate_occurrences() {
        assert_eq!(
            join_mate_occurrences(
                &["sub1".to_string(), "part1".to_string()],
                &["sub2".to_string()],
                None
            ),
            "sub1_SUB_part1_to_sub2"
        );
    }
}
