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

use crate::endpoint::{AsyncClient, Client, Query};
use crate::endpoints::{GetElements, ElementsMap};
use crate::error::{ApiError, OnshapeError};
use crate::model::Element;
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
