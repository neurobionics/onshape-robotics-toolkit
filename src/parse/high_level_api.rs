use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use crate::parse::{
    InstanceTraverser, ParseConfig as RustParseConfig, FetchConfig as RustFetchConfig
};
use crate::parse::models::*;

/// Python-exposed configuration for assembly parsing
#[pyclass]
#[derive(Debug, Clone)]
pub struct ParseConfig {
    #[pyo3(get, set)]
    pub max_depth: usize,
    #[pyo3(get, set)]
    pub configuration: Option<String>,
    #[pyo3(get, set)]
    pub with_mass_properties: bool,
    #[pyo3(get, set)]
    pub include_mate_features: bool,
    #[pyo3(get, set)]
    pub include_mate_connectors: bool,
    #[pyo3(get, set)]
    pub include_non_solids: bool,
    #[pyo3(get, set)]
    pub log_response: bool,
}

#[pymethods]
impl ParseConfig {
    #[new]
    #[pyo3(signature = (max_depth=0, configuration=None, with_mass_properties=true, include_mate_features=true, include_mate_connectors=true, include_non_solids=false, log_response=false))]
    pub fn new(
        max_depth: usize,
        configuration: Option<String>,
        with_mass_properties: bool,
        include_mate_features: bool,
        include_mate_connectors: bool,
        include_non_solids: bool,
        log_response: bool,
    ) -> Self {
        Self {
            max_depth,
            configuration,
            with_mass_properties,
            include_mate_features,
            include_mate_connectors,
            include_non_solids,
            log_response,
        }
    }
}

impl From<&ParseConfig> for RustParseConfig {
    fn from(config: &ParseConfig) -> Self {
        RustParseConfig {
            max_depth: config.max_depth,
            configuration: config.configuration.clone(),
            with_mass_properties: config.with_mass_properties,
            include_mate_features: config.include_mate_features,
            include_mate_connectors: config.include_mate_connectors,
            include_non_solids: config.include_non_solids,
            log_response: config.log_response,
        }
    }
}

/// Python-exposed configuration for async fetching
#[pyclass]
#[derive(Debug, Clone)]
pub struct FetchConfig {
    #[pyo3(get, set)]
    pub max_concurrent_requests: usize,
    #[pyo3(get, set)]
    pub request_timeout_secs: u64,
    #[pyo3(get, set)]
    pub max_retries: usize,
    #[pyo3(get, set)]
    pub retry_base_delay_ms: u64,
    #[pyo3(get, set)]
    pub retry_max_delay_ms: u64,
    #[pyo3(get, set)]
    pub batch_size: usize,
    #[pyo3(get, set)]
    pub batch_delay_ms: u64,
}

#[pymethods]
impl FetchConfig {
    #[new]
    #[pyo3(signature = (max_concurrent_requests=15, request_timeout_secs=60, max_retries=3, retry_base_delay_ms=100, retry_max_delay_ms=5000, batch_size=30, batch_delay_ms=50))]
    pub fn new(
        max_concurrent_requests: usize,
        request_timeout_secs: u64,
        max_retries: usize,
        retry_base_delay_ms: u64,
        retry_max_delay_ms: u64,
        batch_size: usize,
        batch_delay_ms: u64,
    ) -> Self {
        Self {
            max_concurrent_requests,
            request_timeout_secs,
            max_retries,
            retry_base_delay_ms,
            retry_max_delay_ms,
            batch_size,
            batch_delay_ms,
        }
    }
}

impl From<&FetchConfig> for RustFetchConfig {
    fn from(config: &FetchConfig) -> Self {
        RustFetchConfig {
            max_concurrent_requests: config.max_concurrent_requests,
            request_timeout_secs: config.request_timeout_secs,
            max_retries: config.max_retries,
            retry_base_delay_ms: config.retry_base_delay_ms,
            retry_max_delay_ms: config.retry_max_delay_ms,
            batch_size: config.batch_size,
            batch_delay_ms: config.batch_delay_ms,
        }
    }
}

/// Python-exposed parse result
#[pyclass]
#[derive(Debug, Clone)]
pub struct ParseResult {
    #[pyo3(get)]
    pub instances: Py<PyDict>,
    #[pyo3(get)]
    pub occurrences: Py<PyDict>,
    #[pyo3(get)]
    pub id_to_name_map: Py<PyDict>,
    #[pyo3(get)]
    pub subassemblies: Py<PyDict>,
    #[pyo3(get)]
    pub rigid_subassemblies: Py<PyDict>,
    #[pyo3(get)]
    pub parts: Py<PyDict>,
    #[pyo3(get)]
    pub performance_stats: Py<PyDict>,
}

/// Python-exposed assembly parser
#[pyclass]
pub struct RustAssemblyParser {
    access_key: String,
    secret_key: String,
    base_url: String,
    parse_config: ParseConfig,
}

#[pymethods]
impl RustAssemblyParser {
    #[new]
    pub fn new(
        access_key: String,
        secret_key: String,
        base_url: String,
        parse_config: ParseConfig,
    ) -> Self {
        Self {
            access_key,
            secret_key,
            base_url,
            parse_config,
        }
    }

    pub fn parse_assembly_async(&self, assembly: &PyAny, max_depth: usize) -> PyResult<ParseResult> {
        // Convert Python assembly to Rust
        let rust_assembly = convert_assembly_from_python_sync(assembly)?;

        // Create parse config
        let config = RustParseConfig {
            max_depth,
            configuration: self.parse_config.configuration.clone(),
            with_mass_properties: self.parse_config.with_mass_properties,
            include_mate_features: self.parse_config.include_mate_features,
            include_mate_connectors: self.parse_config.include_mate_connectors,
            include_non_solids: self.parse_config.include_non_solids,
            log_response: self.parse_config.log_response,
        };

        // Use sync blocking call
        Python::with_gil(|py| {
            let traversal_result = py.allow_threads(|| {
                let rt = tokio::runtime::Runtime::new().map_err(|e|
                    PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Failed to create runtime: {}", e))
                )?;
                rt.block_on(InstanceTraverser::get_instances(&rust_assembly, config))
                    .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
            })?;

            // Convert results to Python dictionaries
            let instances_dict = PyDict::new(py);
            for (key, instance) in traversal_result.instance_map {
                let instance_py = convert_instance_to_python(py, &instance)?;
                instances_dict.set_item(key, instance_py)?;
            }

            let occurrences_dict = PyDict::new(py);
            for (key, occurrence) in traversal_result.occurrence_map {
                let occurrence_py = convert_occurrence_to_python(py, &occurrence)?;
                occurrences_dict.set_item(key, occurrence_py)?;
            }

            let id_to_name_dict = PyDict::new(py);
            for (key, name) in traversal_result.id_to_name_map {
                id_to_name_dict.set_item(key, name)?;
            }

            // Create empty dicts for now - these would be populated by full implementation
            let empty_dict = PyDict::new(py);
            let performance_dict = PyDict::new(py);
            performance_dict.set_item("parsing_time_ms", 0)?;
            performance_dict.set_item("instances_processed", instances_dict.len())?;

            Ok(ParseResult {
                instances: instances_dict.into(),
                occurrences: occurrences_dict.into(),
                id_to_name_map: id_to_name_dict.into(),
                subassemblies: empty_dict.into(),
                rigid_subassemblies: empty_dict.into(),
                parts: empty_dict.into(),
                performance_stats: performance_dict.into(),
            })
        })
    }

    pub fn cleanup(&self) -> PyResult<()> {
        // Cleanup any resources
        Ok(())
    }
}

/// Main entry point for getting instances from Python
#[pyfunction]
pub fn get_instances_rust(
    py: Python,
    assembly: &PyAny,
    max_depth: Option<usize>,
) -> PyResult<(Py<PyDict>, Py<PyDict>, Py<PyDict>)> {
    let max_depth = max_depth.unwrap_or(0);

    // Convert Python assembly to Rust
    let rust_assembly = convert_assembly_from_python_sync(assembly)?;

    // Create parse config
    let config = RustParseConfig {
        max_depth,
        configuration: None,
        with_mass_properties: false, // Sync version without mass props for simplicity
        include_mate_features: true,
        include_mate_connectors: true,
        include_non_solids: false,
        log_response: false,
    };

    // For now, use sync blocking call - TODO: implement proper async bridge
    let traversal_result = py.allow_threads(|| {
        // Use a simple sync runtime for now
        let rt = tokio::runtime::Runtime::new().map_err(|e|
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Failed to create runtime: {}", e))
        )?;
        rt.block_on(InstanceTraverser::get_instances(&rust_assembly, config))
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    })?;

    // Convert results to Python dictionaries
    let instances_dict = PyDict::new(py);
    for (key, instance) in traversal_result.instance_map {
        let instance_py = convert_instance_to_python(py, &instance)?;
        instances_dict.set_item(key, instance_py)?;
    }

    let occurrences_dict = PyDict::new(py);
    for (key, occurrence) in traversal_result.occurrence_map {
        let occurrence_py = convert_occurrence_to_python(py, &occurrence)?;
        occurrences_dict.set_item(key, occurrence_py)?;
    }

    let id_to_name_dict = PyDict::new(py);
    for (key, name) in traversal_result.id_to_name_map {
        id_to_name_dict.set_item(key, name)?;
    }

    Ok((instances_dict.into(), occurrences_dict.into(), id_to_name_dict.into()))
}

/// Main entry point for getting mates and relations from Python
#[pyfunction]
pub fn get_mates_and_relations_rust(
    py: Python,
    assembly: &PyAny,
    _subassemblies: &PyDict,
    _rigid_subassemblies: &PyDict,
    _id_to_name_map: &PyDict,
    _parts: &PyDict,
) -> PyResult<(Py<PyDict>, Py<PyDict>)> {
    // Convert Python assembly to Rust
    let rust_assembly = convert_assembly_from_python_sync(assembly)?;

    // Process assembly features to extract mates and relations
    let mates_dict = PyDict::new(py);
    let relations_dict = PyDict::new(py);

    // Process root assembly features
    for feature in &rust_assembly.root_assembly.sub_assembly.features {
        match &feature.feature_data {
            AssemblyFeatureData::Mate(mate_data) => {
                let mate_py = convert_mate_to_python(py, mate_data)?;
                mates_dict.set_item(&feature.id, mate_py)?;
            }
            AssemblyFeatureData::MateRelation(relation_data) => {
                let relation_py = convert_mate_relation_to_python(py, relation_data)?;
                relations_dict.set_item(&feature.id, relation_py)?;
            }
            _ => {} // Skip other feature types
        }
    }

    // Also process subassembly features if needed
    for subassembly in &rust_assembly.sub_assemblies {
        for feature in &subassembly.features {
            match &feature.feature_data {
                AssemblyFeatureData::Mate(mate_data) => {
                    let mate_py = convert_mate_to_python(py, mate_data)?;
                    mates_dict.set_item(&feature.id, mate_py)?;
                }
                AssemblyFeatureData::MateRelation(relation_data) => {
                    let relation_py = convert_mate_relation_to_python(py, relation_data)?;
                    relations_dict.set_item(&feature.id, relation_py)?;
                }
                _ => {} // Skip other feature types
            }
        }
    }

    Ok((mates_dict.into(), relations_dict.into()))
}

/// Get performance metrics
#[pyfunction]
pub fn get_performance_metrics(py: Python) -> PyResult<Py<PyDict>> {
    let metrics = PyDict::new(py);
    metrics.set_item("total_requests", 0)?;
    metrics.set_item("cache_hits", 0)?;
    metrics.set_item("cache_misses", 0)?;
    metrics.set_item("avg_request_time_ms", 0.0)?;
    Ok(metrics.into())
}

/// Internal function to convert Python assembly to Rust (synchronous version)
fn convert_assembly_from_python_sync(assembly_py: &PyAny) -> PyResult<Assembly> {
    // Extract basic assembly information from Python object
    let root_assembly_py = assembly_py.getattr("rootAssembly")?;

    // Convert root assembly
    let root_assembly = convert_root_assembly_from_python(root_assembly_py)?;

    // Convert subassemblies if present
    let sub_assemblies = if let Ok(sub_assemblies_py) = assembly_py.getattr("subAssemblies") {
        let sub_assemblies_list: &PyList = sub_assemblies_py.downcast()?;
        sub_assemblies_list.iter()
            .filter_map(|item| convert_subassembly_from_python(item).ok())
            .collect()
    } else {
        Vec::new()
    };

    // Convert parts if present
    let parts = if let Ok(parts_py) = assembly_py.getattr("parts") {
        let parts_list: &PyList = parts_py.downcast()?;
        parts_list.iter()
            .filter_map(|item| convert_part_from_python(item).ok())
            .collect()
    } else {
        Vec::new()
    };

    Ok(Assembly::new(root_assembly, sub_assemblies, parts))
}

/// Convert Python root assembly to Rust
fn convert_root_assembly_from_python(root_assembly_py: &PyAny) -> PyResult<RootAssembly> {
    // Extract basic metadata
    let full_configuration = root_assembly_py.getattr("fullConfiguration")?.extract::<String>()?;
    let configuration = root_assembly_py.getattr("configuration")?.extract::<String>()?;
    let document_id = root_assembly_py.getattr("documentId")?.extract::<String>()?;
    let element_id = root_assembly_py.getattr("elementId")?.extract::<String>()?;
    let document_microversion = root_assembly_py.getattr("documentMicroversion")?.extract::<String>()?;

    // Create base metadata
    let base = BaseMetadata {
        full_configuration,
        configuration,
        document_id,
        element_id,
        document_microversion,
    };

    // Convert instances
    let instances = if let Ok(instances_py) = root_assembly_py.getattr("instances") {
        let instances_list: &PyList = instances_py.downcast()?;
        instances_list.iter()
            .filter_map(|item| convert_instance_from_python(item).ok())
            .collect()
    } else {
        Vec::new()
    };

    // Convert features
    let features = if let Ok(features_py) = root_assembly_py.getattr("features") {
        let features_list: &PyList = features_py.downcast()?;
        features_list.iter()
            .filter_map(|item| convert_feature_from_python(item).ok())
            .collect()
    } else {
        Vec::new()
    };

    // Convert occurrences
    let occurrences = if let Ok(occurrences_py) = root_assembly_py.getattr("occurrences") {
        let occurrences_list: &PyList = occurrences_py.downcast()?;
        occurrences_list.iter()
            .filter_map(|item| convert_occurrence_from_python(item).ok())
            .collect()
    } else {
        Vec::new()
    };

    // Extract mass properties if available
    let mass_property = root_assembly_py.getattr("MassProperty")
        .ok()
        .and_then(|mp| {
            if mp.is_none() {
                None
            } else {
                // For now, return None - mass properties conversion can be added later
                None
            }
        });

    let sub_assembly = SubAssembly {
        base,
        instances,
        features,
    };

    Ok(RootAssembly {
        sub_assembly,
        occurrences,
        mass_property,
    })
}

/// Convert Python subassembly to Rust
fn convert_subassembly_from_python(subassembly_py: &PyAny) -> PyResult<SubAssembly> {
    // Similar to root assembly conversion but simpler
    let full_configuration = subassembly_py.getattr("fullConfiguration")?.extract::<String>()?;
    let configuration = subassembly_py.getattr("configuration")?.extract::<String>()?;
    let document_id = subassembly_py.getattr("documentId")?.extract::<String>()?;
    let element_id = subassembly_py.getattr("elementId")?.extract::<String>()?;
    let document_microversion = subassembly_py.getattr("documentMicroversion")?.extract::<String>()?;

    let base = BaseMetadata {
        full_configuration,
        configuration,
        document_id,
        element_id,
        document_microversion,
    };

    // For simplicity, start with empty instances and features
    // These can be populated from the Python object if needed
    let instances = Vec::new();
    let features = Vec::new();

    Ok(SubAssembly {
        base,
        instances,
        features,
    })
}

/// Convert Python part to Rust
fn convert_part_from_python(part_py: &PyAny) -> PyResult<Part> {
    let part_id = part_py.getattr("partId")?.extract::<String>()?;
    let document_id = part_py.getattr("documentId")?.extract::<String>()?;
    let element_id = part_py.getattr("elementId")?.extract::<String>()?;
    let document_microversion = part_py.getattr("documentMicroversion")?.extract::<String>()?;

    // Extract other basic fields with defaults - use proper PyO3 None handling
    let body_type = match part_py.getattr("bodyType") {
        Ok(attr) if !attr.is_none() => attr.extract::<String>().unwrap_or_else(|_| "solid".to_string()),
        _ => "solid".to_string(),
    };

    let is_standard_content = match part_py.getattr("isStandardContent") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    let base = BaseMetadata {
        full_configuration: "default".to_string(),
        configuration: "default".to_string(),
        document_id,
        element_id,
        document_microversion,
    };

    Ok(Part::new(
        base,
        part_id,
        body_type,
        is_standard_content,
        None, // Mass properties to be set later
    ))
}

/// Convert Python instance to Rust
fn convert_instance_from_python(instance_py: &PyAny) -> PyResult<Instance> {
    let instance_type_str = instance_py.getattr("type")?.extract::<String>()?;
    let id = instance_py.getattr("id")?.extract::<String>()?;
    let name = instance_py.getattr("name")?.extract::<String>()?;

    let suppressed = match instance_py.getattr("suppressed") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    let hidden = match instance_py.getattr("hidden") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    // Extract transform
    let transform = instance_py.getattr("transform")
        .and_then(|t| t.extract::<Vec<f64>>())
        .unwrap_or_else(|_| vec![1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]);

    match instance_type_str.as_str() {
        "Part" => {
            let part_id = instance_py.getattr("partId")?.extract::<String>()?;
            let _is_standard_content = match instance_py.getattr("isStandardContent") {
                Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
                _ => false,
            };

            Ok(Instance::Part(PartInstance::new(
                id,
                name,
                part_id,
                InstanceType::Part,
                suppressed,
                hidden,
                transform,
            )))
        }
        "Assembly" => {
            let document_id = instance_py.getattr("documentId")?.extract::<String>()?;
            let element_id = instance_py.getattr("elementId")?.extract::<String>()?;

            Ok(Instance::Assembly(AssemblyInstance::new(
                id,
                name,
                document_id,
                element_id,
                InstanceType::Assembly,
                suppressed,
                hidden,
                transform,
                None, // is_rigid to be determined later
            )))
        }
        _ => Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Unknown instance type: {}", instance_type_str)))
    }
}

/// Convert Python feature to Rust
fn convert_feature_from_python(feature_py: &PyAny) -> PyResult<AssemblyFeature> {
    let id = feature_py.getattr("id")?.extract::<String>()?;
    let feature_type_str = feature_py.getattr("featureType")?.extract::<String>()?;

    let suppressed = match feature_py.getattr("suppressed") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    let feature_type = match feature_type_str.as_str() {
        "mate" => AssemblyFeatureType::Mate,
        "mateRelation" => AssemblyFeatureType::MateRelation,
        "mateGroup" => AssemblyFeatureType::MateGroup,
        "mateConnector" => AssemblyFeatureType::MateConnector,
        _ => return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Unknown feature type: {}", feature_type_str)))
    };

    // For now, create generic feature data
    let feature_data = AssemblyFeatureData::Generic(serde_json::Value::Null);

    Ok(AssemblyFeature::new(
        id,
        feature_type,
        suppressed,
        feature_data,
    ))
}

/// Convert Python occurrence to Rust
fn convert_occurrence_from_python(occurrence_py: &PyAny) -> PyResult<Occurrence> {
    let path = occurrence_py.getattr("path")?.extract::<Vec<String>>()?;
    let transform = occurrence_py.getattr("transform")
        .and_then(|t| t.extract::<Vec<f64>>())
        .unwrap_or_else(|_| vec![1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]);

    let hidden = match occurrence_py.getattr("hidden") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    let suppressed = match occurrence_py.getattr("suppressed") {
        Ok(attr) if !attr.is_none() => attr.extract::<bool>().unwrap_or(false),
        _ => false,
    };

    Ok(Occurrence {
        path,
        transform,
        hidden,
        suppressed,
    })
}

// Conversion functions from Rust to Python

/// Convert Rust instance to Python
fn convert_instance_to_python<'a>(py: Python<'a>, instance: &Instance) -> PyResult<&'a PyAny> {
    let dict = PyDict::new(py);

    match instance {
        Instance::Part(part_instance) => {
            dict.set_item("id", &part_instance.id)?;
            dict.set_item("name", &part_instance.name)?;
            dict.set_item("part_id", &part_instance.part_id)?;
            dict.set_item("type", "Part")?;
            dict.set_item("suppressed", part_instance.suppressed)?;
            dict.set_item("hidden", part_instance.hidden)?;
            dict.set_item("transform", &part_instance.transform)?;
            dict.set_item("is_standard_content", part_instance.is_standard_content)?;
        }
        Instance::Assembly(assembly_instance) => {
            dict.set_item("id", &assembly_instance.id)?;
            dict.set_item("name", &assembly_instance.name)?;
            dict.set_item("document_id", &assembly_instance.document_id)?;
            dict.set_item("element_id", &assembly_instance.element_id)?;
            dict.set_item("type", "Assembly")?;
            dict.set_item("suppressed", assembly_instance.suppressed)?;
            dict.set_item("hidden", assembly_instance.hidden)?;
            dict.set_item("transform", &assembly_instance.transform)?;
            dict.set_item("is_rigid", assembly_instance.is_rigid)?;
        }
    }

    Ok(dict.as_ref())
}

/// Convert Rust occurrence to Python
fn convert_occurrence_to_python<'a>(py: Python<'a>, occurrence: &Occurrence) -> PyResult<&'a PyAny> {
    let dict = PyDict::new(py);
    dict.set_item("path", &occurrence.path)?;
    dict.set_item("transform", &occurrence.transform)?;
    dict.set_item("hidden", occurrence.hidden)?;
    dict.set_item("suppressed", occurrence.suppressed)?;
    Ok(dict.as_ref())
}

/// Convert Rust mate to Python
fn convert_mate_to_python<'a>(py: Python<'a>, mate: &MateFeatureData) -> PyResult<&'a PyAny> {
    let dict = PyDict::new(py);
    dict.set_item("id", &mate.id)?;
    dict.set_item("name", &mate.name)?;
    dict.set_item("mate_type", format!("{:?}", mate.mate_type))?;

    // Convert mated entities
    let entities_list = PyList::empty(py);
    for entity in &mate.mated_entities {
        let entity_dict = PyDict::new(py);
        entity_dict.set_item("mated_occurrence", &entity.mated_occurrence)?;
        entities_list.append(entity_dict)?;
    }
    dict.set_item("mated_entities", entities_list)?;

    Ok(dict.as_ref())
}

/// Convert Rust mate relation to Python
fn convert_mate_relation_to_python<'a>(py: Python<'a>, relation: &MateRelationFeatureData) -> PyResult<&'a PyAny> {
    let dict = PyDict::new(py);
    dict.set_item("id", &relation.id)?;
    dict.set_item("name", &relation.name)?;
    dict.set_item("relation_type", format!("{:?}", relation.relation_type))?;
    dict.set_item("relation_ratio", relation.relation_ratio)?;
    dict.set_item("relation_length", relation.relation_length)?;

    // Convert mates references
    let mates_list = PyList::empty(py);
    for mate_ref in &relation.mates {
        let mate_dict = PyDict::new(py);
        mate_dict.set_item("feature_id", &mate_ref.feature_id)?;
        mate_dict.set_item("feature_type", &mate_ref.feature_type)?;
        mates_list.append(mate_dict)?;
    }
    dict.set_item("mates", mates_list)?;

    Ok(dict.as_ref())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_config_creation() {
        let config = ParseConfig::new(5, None, true, true, false, false, false);
        assert_eq!(config.max_depth, 5);
        assert!(config.with_mass_properties);
        assert!(!config.include_mate_connectors);
    }

    #[test]
    fn test_fetch_config_creation() {
        let config = FetchConfig::new(20, 30, 5, 200, 10000, 40, 100);
        assert_eq!(config.max_concurrent_requests, 20);
        assert_eq!(config.batch_size, 40);
    }
}
