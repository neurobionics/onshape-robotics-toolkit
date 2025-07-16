#![allow(non_local_definitions)]

use std::time::{SystemTime, UNIX_EPOCH};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::path::Path;

use base64::Engine;
use bytes::Bytes;
use http::{Method, Request, Response};
use hmac::{Hmac, Mac};
use pyo3::prelude::*;
use reqwest::Client as ReqwestClient;
use sha2::Sha256;
use url::Url;

use crate::endpoint::{AsyncClient, Client, Query, Endpoint};
use crate::endpoints::{
    GetElements, ElementsMap, GetDocumentMetadata, GetVariables, SetVariables, VariableUpdate,
    GetAssembly, GetPartMassProperties, GetAssemblyMassProperties, DownloadPartStl
};
use crate::error::{ApiError, OnshapeError};
use crate::model::{Element, DocumentMetaData, Variable, Assembly, RootAssembly, MassProperties};
use crate::utils::generate_nonce;

type HmacSha256 = Hmac<Sha256>;

const DEFAULT_BASE_URL: &str = "https://cad.onshape.com";

/// Synchronous Onshape API client
#[pyclass]
#[derive(Debug, Clone)]
pub struct OnshapeClient {
    access_key: String,
    secret_key: String,
    base_url: Url,
    client: ReqwestClient,
    api_call_count: Arc<AtomicU64>,
}

/// Asynchronous Onshape API client
#[derive(Debug, Clone)]
pub struct AsyncOnshapeClient {
    access_key: String,
    secret_key: String,
    base_url: Url,
    client: ReqwestClient,
    api_call_count: Arc<AtomicU64>,
}

impl OnshapeClient {
    /// Create a new Onshape client
    ///
    /// # Arguments
    /// * `env_file_path` - Optional path to .env file containing credentials
    ///
    /// If no path is provided, credentials will be loaded from system environment variables:
    /// - ONSHAPE_ACCESS_KEY
    /// - ONSHAPE_SECRET_KEY
    pub fn new(env_file_path: Option<&str>) -> Result<Self, OnshapeError> {
        Self::new_with_base_url(env_file_path, DEFAULT_BASE_URL)
    }

    /// Create a new Onshape client with custom base URL
    ///
    /// # Arguments
    /// * `env_file_path` - Optional path to .env file containing credentials
    /// * `base_url` - The base URL for the Onshape API
    pub fn new_with_base_url(
        env_file_path: Option<&str>,
        base_url: &str,
    ) -> Result<Self, OnshapeError> {
        let (access_key, secret_key) = load_credentials(env_file_path)?;

        let base_url = Url::parse(base_url)
            .map_err(|e| OnshapeError::InvalidInput(format!("Invalid base URL: {}", e)))?;

        let client = ReqwestClient::builder()
            .user_agent("Onshape Rust Client")
            .build()
            .map_err(OnshapeError::Http)?;

        Ok(Self {
            access_key,
            secret_key,
            base_url,
            client,
            api_call_count: Arc::new(AtomicU64::new(0)),
        })
    }

    /// Create a new Onshape client with explicit credentials (internal use only)
    ///
    /// This method is kept for internal use and testing purposes.
    /// External users should use new() with .env file or environment variables.
    #[allow(dead_code)]
    pub(crate) fn new_with_credentials(
        access_key: String,
        secret_key: String
    ) -> Result<Self, OnshapeError> {
        Self::new_with_credentials_and_base_url(access_key, secret_key, DEFAULT_BASE_URL)
    }

    /// Create a new Onshape client with explicit credentials and base URL (internal use only)
    #[allow(dead_code)]
    pub(crate) fn new_with_credentials_and_base_url(
        access_key: String,
        secret_key: String,
        base_url: &str,
    ) -> Result<Self, OnshapeError> {
        let base_url = Url::parse(base_url)
            .map_err(|e| OnshapeError::InvalidInput(format!("Invalid base URL: {}", e)))?;

        let client = ReqwestClient::builder()
            .user_agent("Onshape Rust Client")
            .build()
            .map_err(OnshapeError::Http)?;

        Ok(Self {
            access_key,
            secret_key,
            base_url,
            client,
            api_call_count: Arc::new(AtomicU64::new(0)),
        })
    }

    fn make_auth_header(
        &self,
        method: &Method,
        path: &str,
        query: &str,
        date: &str,
        nonce: &str,
        content_type: &str,
    ) -> Result<String, OnshapeError> {
        let string_to_sign = format!(
            "{}\n{}\n{}\n{}\n{}\n{}\n",
            method.as_str(),
            nonce,
            date,
            content_type,
            path,
            query
        ).to_lowercase(); // Convert the entire string to lowercase, like Python does

        let mut mac = HmacSha256::new_from_slice(self.secret_key.as_bytes())
            .map_err(|e| OnshapeError::Auth(format!("Invalid secret key: {}", e)))?;

        mac.update(string_to_sign.as_bytes());
        let signature = mac.finalize().into_bytes();
        let signature_b64 = base64::engine::general_purpose::STANDARD.encode(signature);

        Ok(format!("On {}:HmacSHA256:{}", self.access_key, signature_b64))
    }
}

/// Load credentials from .env file or environment variables
fn load_credentials(env_file_path: Option<&str>) -> Result<(String, String), OnshapeError> {
    match env_file_path {
        Some(path) => {
            // Load from specified .env file
            if !Path::new(path).exists() {
                return Err(OnshapeError::InvalidInput(
                    format!("Environment file not found: {}", path)
                ));
            }

            dotenv::from_path(path)
                .map_err(|e| OnshapeError::InvalidInput(format!("Failed to load .env file: {}", e)))?;

            println!("Loaded Onshape credentials from: {}", path);
        }
        None => {
            // Try to load from default .env file, but don't fail if it doesn't exist
            let _ = dotenv::dotenv();
            println!("Loaded Onshape credentials from environment variables");
        }
    }

    let access_key = std::env::var("ONSHAPE_ACCESS_KEY")
        .map_err(|_| OnshapeError::Auth("ONSHAPE_ACCESS_KEY not found in environment".to_string()))?;
    let secret_key = std::env::var("ONSHAPE_SECRET_KEY")
        .map_err(|_| OnshapeError::Auth("ONSHAPE_SECRET_KEY not found in environment".to_string()))?;

    Ok((access_key, secret_key))
}

#[pymethods]
impl OnshapeClient {
    #[new]
    #[pyo3(signature = (env_file_path=None, base_url=None))]
    pub fn py_new(
        env_file_path: Option<String>,
        base_url: Option<String>,
    ) -> PyResult<Self> {
        let base_url = base_url.as_deref().unwrap_or(DEFAULT_BASE_URL);
        Self::new_with_base_url(env_file_path.as_deref(), base_url)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
    }

    #[getter]
    pub fn base_url(&self) -> String {
        self.base_url.to_string()
    }

    #[setter]
    pub fn set_base_url(&mut self, base_url: String) -> PyResult<()> {
        self.base_url = Url::parse(&base_url)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(format!("Invalid URL: {}", e)))?;
        Ok(())
    }

    /// Get a list of all elements in a document
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///
    /// Returns:
    ///     A dictionary mapping element names to Element objects
    ///
    /// Raises:
    ///     ValueError: If the request fails or returns an error status
    pub fn get_elements(
        &self,
        did: &str,
        wtype: &str,
        wid: &str
    ) -> PyResult<HashMap<String, Element>> {
        let endpoint = GetElements {
            did,
            wtype,
            wid,
        };

        let elements_map: ElementsMap = Query::query(&endpoint, self)
            .map_err(|e| {
                pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e))
            })?;

        Ok(elements_map.into())
    }

    /// Get document metadata for a specified document
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///
    /// Returns:
    ///     DocumentMetaData object containing document information
    ///
    /// Raises:
    ///     ValueError: If the document is not found or access is forbidden
    pub fn get_document_metadata(&self, did: &str) -> PyResult<DocumentMetaData> {
        if did.len() != 24 {
            return Err(pyo3::exceptions::PyValueError::new_err(format!("Invalid document ID: {}", did)));
        }

        let endpoint = GetDocumentMetadata { did };
        let document: DocumentMetaData = Query::query(&endpoint, self)
            .map_err(|e| match e {
                ApiError::Server(msg) if msg.contains("Not Found (404)") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Document does not exist: {}", did))
                }
                ApiError::Auth(msg) if msg.contains("Unauthorized") || msg.contains("Forbidden") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Access forbidden for document: {}", did))
                }
                // Fallback for old error patterns
                ApiError::Server(msg) if msg.contains("404") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Document does not exist: {}", did))
                }
                ApiError::Auth(_) => {
                    pyo3::exceptions::PyValueError::new_err(format!("Access forbidden for document: {}", did))
                }
                ApiError::Server(msg) if msg.contains("403") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Access forbidden for document: {}", did))
                }
                _ => pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e))
            })?;

        Ok(document)
    }

    /// Get a list of variables in a variable studio within a document
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the variable studio
    ///
    /// Returns:
    ///     A dictionary mapping variable names to Variable objects
    ///
    /// Raises:
    ///     RuntimeError: If the request fails
    pub fn get_variables(
        &self,
        did: &str,
        wid: &str,
        eid: &str
    ) -> PyResult<HashMap<String, Variable>> {
        let endpoint = GetVariables { did, wid, eid };

        // The API returns an array with an object containing a "variables" array
        let response: Vec<serde_json::Value> = Query::query(&endpoint, self)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e)))?;

        if response.is_empty() {
            return Ok(HashMap::new());
        }

        // Parse the variables from the response
        let variables = response[0]["variables"].as_array()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("Invalid variables response format"))?;

        let mut variables_map = HashMap::new();
        for var_value in variables {
            let variable: Variable = serde_json::from_value(var_value.clone())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to parse variable: {}", e)))?;
            variables_map.insert(variable.name.clone(), variable);
        }

        Ok(variables_map)
    }

    /// Set values for variables of a variable studio in a document
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the variable studio
    ///     variables: A dictionary of variable name and expression pairs
    ///
    /// Returns:
    ///     True if successful
    ///
    /// Raises:
    ///     RuntimeError: If the request fails
    pub fn set_variables(
        &self,
        did: &str,
        wid: &str,
        eid: &str,
        variables: HashMap<String, String>
    ) -> PyResult<bool> {
        let variable_updates: Vec<VariableUpdate> = variables
            .iter()
            .map(|(name, expression)| VariableUpdate {
                name: name.as_str(),
                expression: expression.as_str(),
            })
            .collect();

        let endpoint = SetVariables {
            did,
            wid,
            eid,
            variables: variable_updates,
        };

        let _: serde_json::Value = Query::query(&endpoint, self)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e)))?;

        Ok(true)
    }

    /// Get assembly data for a specified document / workspace / assembly
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the assembly
    ///     configuration: The configuration of the assembly (default: "default")
    ///
    /// Returns:
    ///     Assembly object containing the assembly data
    ///
    /// Raises:
    ///     RuntimeError: If the request fails
    pub fn get_assembly(
        &self,
        did: &str,
        wtype: &str,
        wid: &str,
        eid: &str,
        configuration: Option<&str>
    ) -> PyResult<Assembly> {
        let config = configuration.unwrap_or("default");

        let endpoint = GetAssembly {
            did,
            wtype,
            wid,
            eid,
            configuration: config,
            include_mate_features: true,
            include_mate_connectors: true,
            include_non_solids: false,
        };

        let mut assembly: Assembly = Query::query(&endpoint, self)
            .map_err(|e| match e {
                ApiError::Server(msg) if msg.contains("Not Found (404)") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Assembly not found: {}", did))
                }
                ApiError::Auth(msg) if msg.contains("Unauthorized") || msg.contains("Forbidden") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Unauthorized access to document: {}", did))
                }
                // Fallback for old error patterns
                ApiError::Server(msg) if msg.contains("404") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Assembly not found: {}", did))
                }
                ApiError::Auth(_) => {
                    pyo3::exceptions::PyValueError::new_err(format!("Unauthorized access to document: {}", did))
                }
                ApiError::Server(msg) if msg.contains("401") || msg.contains("403") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Unauthorized access to document: {}", did))
                }
                _ => pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e))
            })?;

        // Set document information
        assembly.document = Some(crate::model::Document {
            did: did.to_string(),
            wtype: wtype.to_string(),
            wid: wid.to_string(),
            eid: eid.to_string(),
            name: None,
            url: None,
        });

        Ok(assembly)
    }

    /// Get root assembly data for a specified document / workspace / element
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the element
    ///     configuration: The configuration of the assembly (default: "default")
    ///     with_mass_properties: Whether to include mass properties (default: false)
    ///
    /// Returns:
    ///     RootAssembly object containing the root assembly data
    ///
    /// Raises:
    ///     RuntimeError: If the request fails
    pub fn get_root_assembly(
        &self,
        did: &str,
        wtype: &str,
        wid: &str,
        eid: &str,
        configuration: Option<&str>,
        with_mass_properties: Option<bool>
    ) -> PyResult<RootAssembly> {
        let assembly = self.get_assembly(did, wtype, wid, eid, configuration)?;
        let mut root_assembly = assembly.root_assembly;

        if with_mass_properties.unwrap_or(false) {
            match self.get_assembly_mass_properties(did, wtype, wid, eid) {
                Ok(mass_props) => {
                    root_assembly.mass_property = Some(mass_props);
                }
                Err(_) => {
                    // Mass properties are optional, so we don't fail if they're not available
                }
            }
        }

        // Add document metadata
        match self.get_document_metadata(did) {
            Ok(doc_meta) => {
                root_assembly.document_meta_data = Some(doc_meta);
            }
            Err(_) => {
                // Document metadata is optional, so we don't fail if it's not available
            }
        }

        Ok(root_assembly)
    }

    /// Get mass properties of a rigid assembly in a document
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the rigid assembly
    ///
    /// Returns:
    ///     MassProperties object containing the mass properties of the assembly
    ///
    /// Raises:
    ///     ValueError: If the assembly does not have mass properties
    ///     RuntimeError: If the request fails
    pub fn get_assembly_mass_properties(
        &self,
        did: &str,
        wtype: &str,
        wid: &str,
        eid: &str
    ) -> PyResult<MassProperties> {
        let endpoint = GetAssemblyMassProperties { did, wtype, wid, eid };

        let mass_props: MassProperties = Query::query(&endpoint, self)
            .map_err(|e| match e {
                ApiError::Server(msg) if msg.contains("Not Found (404)") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Assembly does not have a mass property"))
                }
                // Fallback for old error patterns
                ApiError::Server(msg) if msg.contains("404") => {
                    pyo3::exceptions::PyValueError::new_err(format!("Assembly does not have a mass property"))
                }
                _ => pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e))
            })?;

        Ok(mass_props)
    }

    /// Get mass properties of a part in a part studio
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the element
    ///     part_id: The identifier of the part
    ///
    /// Returns:
    ///     MassProperties object containing the mass properties of the part
    ///
    /// Raises:
    ///     ValueError: If the part does not have a material assigned or is not found
    ///     RuntimeError: If the request fails
    pub fn get_mass_property(
        &self,
        did: &str,
        wtype: &str,
        wid: &str,
        eid: &str,
        part_id: &str
    ) -> PyResult<MassProperties> {
        let endpoint = GetPartMassProperties {
            did,
            wtype,
            wid,
            eid,
            part_id,
            use_mass_properties_overrides: true,
        };

        let response: serde_json::Value = Query::query(&endpoint, self)
            .map_err(|e| match e {
                ApiError::Server(msg) if msg.contains("Not Found (404)") => {
                    pyo3::exceptions::PyValueError::new_err(
                        format!("Part does not have a material assigned or the part is not found")
                    )
                }
                ApiError::Server(msg) if msg.contains("Too Many Requests (429)") => {
                    // Extract retry information from the enhanced error message
                    if msg.contains("retry after") {
                        let retry_msg = msg.split("retry after").nth(1)
                            .and_then(|s| s.split_whitespace().next())
                            .map(|s| format!("Too many requests, please retry after {} seconds", s))
                            .unwrap_or_else(|| "Too many requests, please retry later".to_string());
                        pyo3::exceptions::PyValueError::new_err(retry_msg)
                    } else {
                        pyo3::exceptions::PyValueError::new_err("Too many requests, please retry later")
                    }
                }
                // Fallback for old error patterns
                ApiError::Server(msg) if msg.contains("404") => {
                    pyo3::exceptions::PyValueError::new_err(
                        format!("Part does not have a material assigned or the part is not found")
                    )
                }
                ApiError::Server(msg) if msg.contains("429") => {
                    // Extract retry-after if available
                    pyo3::exceptions::PyValueError::new_err("Too many requests, please retry later")
                }
                _ => pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e))
            })?;

        // Parse the mass properties from the bodies field
        let bodies = response["bodies"].as_object()
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Bodies not found in response"))?;

        let mass_props_value = bodies.get(part_id)
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(format!("Bodies not found in response, broken part? {}", part_id)))?;

        let mass_props: MassProperties = serde_json::from_value(mass_props_value.clone())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to parse mass properties: {}", e)))?;

        Ok(mass_props)
    }

    /// Download an STL file from a part studio as bytes
    ///
    /// Args:
    ///     did: The unique identifier of the document
    ///     wtype: The type of workspace (w, v, or m)
    ///     wid: The unique identifier of the workspace
    ///     eid: The unique identifier of the element
    ///     part_id: The unique identifier of the part
    ///
    /// Returns:
    ///     Bytes containing the STL file content
    ///
    /// Raises:
    ///     RuntimeError: If the download fails
    pub fn download_part_stl(
        &self,
        did: &str,
        wtype: &str,
        wid: &str,
        eid: &str,
        part_id: &str
    ) -> PyResult<Vec<u8>> {
        let endpoint = DownloadPartStl {
            did,
            wtype,
            wid,
            eid,
            part_id,
            mode: "binary",
            grouping: true,
            units: "meter",
        };

        // Build the URL using the Client trait implementation
        let url = Client::rest_endpoint(self, &endpoint.endpoint())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e)))?;

        let mut request = http::Request::builder()
            .method(endpoint.method())
            .uri(url.as_str())
            .header("Accept", "application/vnd.onshape.v1+octet-stream");

        // Add query parameters
        let params = endpoint.parameters();
        if !params.is_empty() {
            let mut url_with_params = url.clone();
            {
                let mut query_pairs = url_with_params.query_pairs_mut();
                for (key, value) in params.iter() {
                    query_pairs.append_pair(key, value);
                }
            }
            request = request.uri(url_with_params.as_str());
        }

        let request = request.body(Vec::new())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to build request: {}", e)))?;

        let response = Client::rest(self, request)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("API error: {}", e)))?;

        if !response.status().is_success() {
            return Err(pyo3::exceptions::PyRuntimeError::new_err(
                format!("Failed to download STL file: {} - {}", response.status(),
                        String::from_utf8_lossy(&response.body()))
            ));
        }

        Ok(response.into_body().to_vec())
    }

    /// Get the current API call count
    ///
    /// Returns:
    ///     The number of API calls made by this client instance
    #[getter]
    pub fn api_call_count(&self) -> u64 {
        self.api_call_count.load(Ordering::Relaxed)
    }
}

impl Client for OnshapeClient {
    type Error = OnshapeError;

    fn rest_endpoint(&self, endpoint: &str) -> Result<Url, ApiError<Self::Error>> {
        self.base_url
            .join(endpoint)
            .map_err(|e| ApiError::Client(OnshapeError::InvalidInput(format!("Invalid endpoint: {}", e))))
    }

    fn rest(&self, request: Request<Vec<u8>>) -> Result<Response<Bytes>, ApiError<Self::Error>> {
        // Increment API call counter
        self.api_call_count.fetch_add(1, Ordering::Relaxed);

        let uri = request.uri();
        let path = uri.path();
        let query = uri.query().unwrap_or("");

        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let date = httpdate::fmt_http_date(SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(now));
        let nonce = generate_nonce();

        let content_type = request
            .headers()
            .get("content-type")
            .and_then(|v| v.to_str().ok())
            .unwrap_or("application/json");

        let auth_header = self
            .make_auth_header(request.method(), path, query, &date, &nonce, content_type)
            .map_err(ApiError::Client)?;

        let mut req_builder = self.client
            .request(request.method().clone(), uri.to_string())
            .header("Date", date)
            .header("On-Nonce", nonce)
            .header("Authorization", auth_header)
            .header("Content-Type", content_type)
            .header("Accept", "application/json");

        let body = request.into_body();
        if !body.is_empty() {
            req_builder = req_builder.body(body);
        }

        // Use blocking call for sync implementation
        let rt = tokio::runtime::Runtime::new()
            .map_err(|e| ApiError::Client(OnshapeError::Io(e)))?;

        let response = rt.block_on(async {
            req_builder.send().await
        }).map_err(|e| ApiError::Client(OnshapeError::Http(e)))?;

        let status = response.status();
        let headers = response.headers().clone();
        let body = rt.block_on(async {
            response.bytes().await
        }).map_err(|e| ApiError::Client(OnshapeError::Http(e)))?;

        let mut resp_builder = Response::builder().status(status);
        for (name, value) in headers.iter() {
            resp_builder = resp_builder.header(name, value);
        }

        resp_builder
            .body(body)
            .map_err(|e| ApiError::Server(e.to_string()))
    }
}

impl AsyncOnshapeClient {
    /// Create a new async Onshape client
    ///
    /// # Arguments
    /// * `env_file_path` - Optional path to .env file containing credentials
    ///
    /// If no path is provided, credentials will be loaded from system environment variables:
    /// - ONSHAPE_ACCESS_KEY
    /// - ONSHAPE_SECRET_KEY
    pub fn new(env_file_path: Option<&str>) -> Result<Self, OnshapeError> {
        Self::new_with_base_url(env_file_path, DEFAULT_BASE_URL)
    }

    /// Create a new async Onshape client with custom base URL
    ///
    /// # Arguments
    /// * `env_file_path` - Optional path to .env file containing credentials
    /// * `base_url` - The base URL for the Onshape API
    pub fn new_with_base_url(
        env_file_path: Option<&str>,
        base_url: &str,
    ) -> Result<Self, OnshapeError> {
        let (access_key, secret_key) = load_credentials(env_file_path)?;

        let base_url = Url::parse(base_url)
            .map_err(|e| OnshapeError::InvalidInput(format!("Invalid base URL: {}", e)))?;

        let client = ReqwestClient::builder()
            .user_agent("Onshape Rust Async Client")
            .build()
            .map_err(OnshapeError::Http)?;

        Ok(Self {
            access_key,
            secret_key,
            base_url,
            client,
            api_call_count: Arc::new(AtomicU64::new(0)),
        })
    }

    /// Create a new async Onshape client with explicit credentials (internal use only)
    ///
    /// This method is kept for internal use and testing purposes.
    /// External users should use new() with .env file or environment variables.
    #[allow(dead_code)]
    pub(crate) fn new_with_credentials(
        access_key: String,
        secret_key: String
    ) -> Result<Self, OnshapeError> {
        Self::new_with_credentials_and_base_url(access_key, secret_key, DEFAULT_BASE_URL)
    }

    /// Create a new async Onshape client with explicit credentials and base URL (internal use only)
    #[allow(dead_code)]
    pub(crate) fn new_with_credentials_and_base_url(
        access_key: String,
        secret_key: String,
        base_url: &str,
    ) -> Result<Self, OnshapeError> {
        let base_url = Url::parse(base_url)
            .map_err(|e| OnshapeError::InvalidInput(format!("Invalid base URL: {}", e)))?;

        let client = ReqwestClient::builder()
            .user_agent("Onshape Rust Async Client")
            .build()
            .map_err(OnshapeError::Http)?;

        Ok(Self {
            access_key,
            secret_key,
            base_url,
            client,
            api_call_count: Arc::new(AtomicU64::new(0)),
        })
    }

    fn make_auth_header(
        &self,
        method: &Method,
        path: &str,
        query: &str,
        date: &str,
        nonce: &str,
        content_type: &str,
    ) -> Result<String, OnshapeError> {
        let string_to_sign = format!(
            "{}\n{}\n{}\n{}\n{}\n{}\n",
            method.as_str(),
            nonce,
            date,
            content_type,
            path,
            query
        ).to_lowercase(); // Convert the entire string to lowercase, like Python does

        let mut mac = HmacSha256::new_from_slice(self.secret_key.as_bytes())
            .map_err(|e| OnshapeError::Auth(format!("Invalid secret key: {}", e)))?;

        mac.update(string_to_sign.as_bytes());
        let signature = mac.finalize().into_bytes();
        let signature_b64 = base64::engine::general_purpose::STANDARD.encode(signature);

        Ok(format!("On {}:HmacSHA256:{}", self.access_key, signature_b64))
    }

    /// Get the current API call count
    pub fn api_call_count(&self) -> u64 {
        self.api_call_count.load(Ordering::Relaxed)
    }
}

#[async_trait::async_trait]
impl AsyncClient for AsyncOnshapeClient {
    type Error = OnshapeError;

    fn rest_endpoint(&self, endpoint: &str) -> Result<Url, ApiError<Self::Error>> {
        self.base_url
            .join(endpoint)
            .map_err(|e| ApiError::Client(OnshapeError::InvalidInput(format!("Invalid endpoint: {}", e))))
    }

    async fn rest(&self, request: Request<Vec<u8>>) -> Result<Response<Bytes>, ApiError<Self::Error>> {
        // Increment API call counter
        self.api_call_count.fetch_add(1, Ordering::Relaxed);

        let uri = request.uri();
        let path = uri.path();
        let query = uri.query().unwrap_or("");

        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let date = httpdate::fmt_http_date(SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(now));
        let nonce = generate_nonce();

        let content_type = request
            .headers()
            .get("content-type")
            .and_then(|v| v.to_str().ok())
            .unwrap_or("application/json");

        let auth_header = self
            .make_auth_header(request.method(), path, query, &date, &nonce, content_type)
            .map_err(ApiError::Client)?;

        let mut req_builder = self.client
            .request(request.method().clone(), uri.to_string())
            .header("Date", date)
            .header("On-Nonce", nonce)
            .header("Authorization", auth_header)
            .header("Content-Type", content_type)
            .header("Accept", "application/json");

        let body = request.into_body();
        if !body.is_empty() {
            req_builder = req_builder.body(body);
        }

        let response = req_builder
            .send()
            .await
            .map_err(|e| ApiError::Client(OnshapeError::Http(e)))?;

        let status = response.status();
        let headers = response.headers().clone();
        let body = response
            .bytes()
            .await
            .map_err(|e| ApiError::Client(OnshapeError::Http(e)))?;

        let mut resp_builder = Response::builder().status(status);
        for (name, value) in headers.iter() {
            resp_builder = resp_builder.header(name, value);
        }

        resp_builder
            .body(body)
            .map_err(|e| ApiError::Server(e.to_string()))
    }
}
