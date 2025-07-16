use std::borrow::Cow;
use std::error::Error;
use std::collections::HashMap;

use bytes::Bytes;
use http::{Method, Request, Response};
use serde::de::DeserializeOwned;
use url::Url;
use async_trait::async_trait;

use crate::error::{ApiError, BodyError};
use crate::params::QueryParams;

/// Trait for defining API endpoints
pub trait Endpoint {
    /// HTTP method for the endpoint
    fn method(&self) -> Method;

    /// The endpoint path (without base URL)
    fn endpoint(&self) -> Cow<'static, str>;

    /// Query parameters for the endpoint
    fn parameters(&self) -> QueryParams {
        QueryParams::default()
    }

    /// Request body for the endpoint
    fn body(&self) -> Result<Option<(&'static str, Vec<u8>)>, BodyError> {
        Ok(None)
    }
}

/// Trait for types that can provide a key for HashMap conversion
pub trait HasKey {
    fn key(&self) -> String;
}

/// Trait for HTTP clients
pub trait Client {
    type Error: Error + Send + Sync + 'static;

    /// Build the complete URL for an endpoint
    fn rest_endpoint(&self, endpoint: &str) -> Result<Url, ApiError<Self::Error>>;

    /// Execute an HTTP request
    fn rest(
        &self,
        request: Request<Vec<u8>>,
    ) -> Result<Response<Bytes>, ApiError<Self::Error>>;
}

/// Trait for executing queries synchronously
pub trait Query<T, C>
where
    C: Client,
{
    fn query(&self, client: &C) -> Result<T, ApiError<C::Error>>;
}

/// Trait for async HTTP clients
#[async_trait]
pub trait AsyncClient {
    type Error: Error + Send + Sync + 'static;

    /// Build the complete URL for an endpoint
    fn rest_endpoint(&self, endpoint: &str) -> Result<Url, ApiError<Self::Error>>;

    /// Execute an HTTP request asynchronously
    async fn rest(
        &self,
        request: Request<Vec<u8>>,
    ) -> Result<Response<Bytes>, ApiError<Self::Error>>;
}

/// Trait for executing queries asynchronously
#[async_trait]
pub trait AsyncQuery<T, C>
where
    C: AsyncClient,
{
    async fn query(&self, client: &C) -> Result<T, ApiError<C::Error>>;
}

// Blanket implementation for any endpoint to return any deserializable type
impl<E, T, C> Query<T, C> for E
where
    E: Endpoint,
    T: DeserializeOwned,
    C: Client,
{
    fn query(&self, client: &C) -> Result<T, ApiError<C::Error>> {
        let url = client.rest_endpoint(&self.endpoint())?;

        let mut request = Request::builder()
            .method(self.method())
            .uri(url.as_str());

        // Add query parameters
        let params = self.parameters();
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

        // Add body if present
        let body = if let Some((content_type, body_data)) = self.body().map_err(ApiError::Body)? {
            request = request.header("Content-Type", content_type);
            body_data
        } else {
            Vec::new()
        };

        let request = request.body(body).map_err(|e| {
            ApiError::Server(format!("Failed to build request: {}", e))
        })?;

        let response = client.rest(request)?;

        let status = response.status();
        let headers = response.headers().clone();
        let body = response.into_body();

        if !status.is_success() {
            let error_msg = String::from_utf8_lossy(&body);
            let retry_after = crate::error::extract_retry_after(&headers);

            return match status.as_u16() {
                401 => Err(ApiError::Auth(format!("Unauthorized: {}", error_msg))),
                403 => Err(ApiError::Auth(format!("Forbidden: {}", error_msg))),
                400 => Err(ApiError::Server(format!("Bad Request (400): {}", error_msg))),
                404 => Err(ApiError::Server(format!("Not Found (404): {}", error_msg))),
                405 => Err(ApiError::Server(format!("Method Not Allowed (405): {}", error_msg))),
                406 => Err(ApiError::Server(format!("Not Acceptable (406): {}", error_msg))),
                409 => Err(ApiError::Server(format!("Conflict (409): {}", error_msg))),
                415 => Err(ApiError::Server(format!("Unsupported Media Type (415): {}", error_msg))),
                429 => {
                    if let Some(seconds) = retry_after {
                        Err(ApiError::Server(format!("Too Many Requests (429): retry after {} seconds - {}", seconds, error_msg)))
                    } else {
                        Err(ApiError::Server(format!("Too Many Requests (429): {}", error_msg)))
                    }
                },
                499 => Err(ApiError::Server(format!("Timeout (499): {}", error_msg))),
                500 => Err(ApiError::Server(format!("Internal Server Error (500): {}", error_msg))),
                503 => {
                    if let Some(seconds) = retry_after {
                        Err(ApiError::Server(format!("Service Unavailable (503): retry after {} seconds - {}", seconds, error_msg)))
                    } else {
                        Err(ApiError::Server(format!("Service Unavailable (503): {}", error_msg)))
                    }
                },
                307 => Err(ApiError::Server(format!("Temporary Redirect (307): {}", error_msg))),
                _ if (500..600).contains(&status.as_u16()) => Err(ApiError::Server(format!("Server Error ({}): {}", status, error_msg))),
                _ => Err(ApiError::Server(format!("HTTP Error ({}): {}", status, error_msg))),
            };
        }

        let json_value: serde_json::Value = serde_json::from_slice(&body)
            .map_err(ApiError::data_type::<T>)?;

        serde_json::from_value::<T>(json_value).map_err(ApiError::data_type::<T>)
    }
}

// Blanket implementation for async queries
#[async_trait]
impl<E, T, C> AsyncQuery<T, C> for E
where
    E: Endpoint + Sync,
    T: DeserializeOwned + Send,
    C: AsyncClient + Sync,
{
    async fn query(&self, client: &C) -> Result<T, ApiError<C::Error>> {
        let url = client.rest_endpoint(&self.endpoint())?;

        let mut request = Request::builder()
            .method(self.method())
            .uri(url.as_str());

        // Add query parameters
        let params = self.parameters();
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

        // Add body if present
        let body = if let Some((content_type, body_data)) = self.body().map_err(ApiError::Body)? {
            request = request.header("Content-Type", content_type);
            body_data
        } else {
            Vec::new()
        };

        let request = request.body(body).map_err(|e| {
            ApiError::Server(e.to_string())
        })?;

        let response = client.rest(request).await?;

        let status = response.status();
        let headers = response.headers().clone();
        let body = response.into_body();

        if !status.is_success() {
            let error_msg = String::from_utf8_lossy(&body);
            let retry_after = crate::error::extract_retry_after(&headers);

            return match status.as_u16() {
                401 => Err(ApiError::Auth(format!("Unauthorized: {}", error_msg))),
                403 => Err(ApiError::Auth(format!("Forbidden: {}", error_msg))),
                400 => Err(ApiError::Server(format!("Bad Request (400): {}", error_msg))),
                404 => Err(ApiError::Server(format!("Not Found (404): {}", error_msg))),
                405 => Err(ApiError::Server(format!("Method Not Allowed (405): {}", error_msg))),
                406 => Err(ApiError::Server(format!("Not Acceptable (406): {}", error_msg))),
                409 => Err(ApiError::Server(format!("Conflict (409): {}", error_msg))),
                415 => Err(ApiError::Server(format!("Unsupported Media Type (415): {}", error_msg))),
                429 => {
                    if let Some(seconds) = retry_after {
                        Err(ApiError::Server(format!("Too Many Requests (429): retry after {} seconds - {}", seconds, error_msg)))
                    } else {
                        Err(ApiError::Server(format!("Too Many Requests (429): {}", error_msg)))
                    }
                },
                499 => Err(ApiError::Server(format!("Timeout (499): {}", error_msg))),
                500 => Err(ApiError::Server(format!("Internal Server Error (500): {}", error_msg))),
                503 => {
                    if let Some(seconds) = retry_after {
                        Err(ApiError::Server(format!("Service Unavailable (503): retry after {} seconds - {}", seconds, error_msg)))
                    } else {
                        Err(ApiError::Server(format!("Service Unavailable (503): {}", error_msg)))
                    }
                },
                307 => Err(ApiError::Server(format!("Temporary Redirect (307): {}", error_msg))),
                _ if (500..600).contains(&status.as_u16()) => Err(ApiError::Server(format!("Server Error ({}): {}", status, error_msg))),
                _ => Err(ApiError::Server(format!("HTTP Error ({}): {}", status, error_msg))),
            };
        }

        let json_value: serde_json::Value = serde_json::from_slice(&body)
            .map_err(ApiError::data_type::<T>)?;

        serde_json::from_value::<T>(json_value).map_err(ApiError::data_type::<T>)
    }
}

/// Helper function to ignore the result of an endpoint and just check for success
pub fn ignore<E>(endpoint: E) -> IgnoreEndpoint<E>
where
    E: Endpoint,
{
    IgnoreEndpoint { endpoint }
}

pub struct IgnoreEndpoint<E> {
    endpoint: E,
}

impl<E, C> Query<(), C> for IgnoreEndpoint<E>
where
    E: Endpoint,
    C: Client,
{
    fn query(&self, client: &C) -> Result<(), ApiError<C::Error>> {
        let _: serde_json::Value = self.endpoint.query(client)?;
        Ok(())
    }
}

#[async_trait]
impl<E, C> AsyncQuery<(), C> for IgnoreEndpoint<E>
where
    E: Endpoint + Sync,
    C: AsyncClient + Sync,
{
    async fn query(&self, client: &C) -> Result<(), ApiError<C::Error>> {
        let _: serde_json::Value = AsyncQuery::query(&self.endpoint, client).await?;
        Ok(())
    }
}

/// Helper function to execute a query that returns an array and converts it to a HashMap
pub fn query_array_to_map<E, T, C, F>(
    endpoint: &E,
    client: &C,
    key_fn: F,
) -> Result<HashMap<String, T>, ApiError<C::Error>>
where
    E: Endpoint,
    T: DeserializeOwned,
    C: Client,
    F: Fn(&T) -> String,
{
    let url = client.rest_endpoint(&endpoint.endpoint())?;

    let mut request = Request::builder()
        .method(endpoint.method())
        .uri(url.as_str());

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

    // Add body if present
    let body = if let Some((content_type, body_data)) = endpoint.body().map_err(ApiError::Body)? {
        request = request.header("Content-Type", content_type);
        body_data
    } else {
        Vec::new()
    };

    let request = request.body(body).map_err(|e| {
        ApiError::Server(format!("Failed to build request: {}", e))
    })?;

    let response = client.rest(request)?;

    let status = response.status();
    let headers = response.headers().clone();
    let body = response.into_body();

    if !status.is_success() {
        let error_msg = String::from_utf8_lossy(&body);
        let retry_after = crate::error::extract_retry_after(&headers);

        return match status.as_u16() {
            401 => Err(ApiError::Auth(format!("Unauthorized: {}", error_msg))),
            403 => Err(ApiError::Auth(format!("Forbidden: {}", error_msg))),
            400 => Err(ApiError::Server(format!("Bad Request (400): {}", error_msg))),
            404 => Err(ApiError::Server(format!("Not Found (404): {}", error_msg))),
            405 => Err(ApiError::Server(format!("Method Not Allowed (405): {}", error_msg))),
            406 => Err(ApiError::Server(format!("Not Acceptable (406): {}", error_msg))),
            409 => Err(ApiError::Server(format!("Conflict (409): {}", error_msg))),
            415 => Err(ApiError::Server(format!("Unsupported Media Type (415): {}", error_msg))),
            429 => {
                if let Some(seconds) = retry_after {
                    Err(ApiError::Server(format!("Too Many Requests (429): retry after {} seconds - {}", seconds, error_msg)))
                } else {
                    Err(ApiError::Server(format!("Too Many Requests (429): {}", error_msg)))
                }
            },
            499 => Err(ApiError::Server(format!("Timeout (499): {}", error_msg))),
            500 => Err(ApiError::Server(format!("Internal Server Error (500): {}", error_msg))),
            503 => {
                if let Some(seconds) = retry_after {
                    Err(ApiError::Server(format!("Service Unavailable (503): retry after {} seconds - {}", seconds, error_msg)))
                } else {
                    Err(ApiError::Server(format!("Service Unavailable (503): {}", error_msg)))
                }
            },
            307 => Err(ApiError::Server(format!("Temporary Redirect (307): {}", error_msg))),
            _ if (500..600).contains(&status.as_u16()) => Err(ApiError::Server(format!("Server Error ({}): {}", status, error_msg))),
            _ => Err(ApiError::Server(format!("HTTP Error ({}): {}", status, error_msg))),
        };
    }

    // Parse JSON array and convert to HashMap using the provided key function
    let items_array: Vec<T> = serde_json::from_slice(&body)
        .map_err(ApiError::data_type::<Vec<T>>)?;

    let items_map = items_array
        .into_iter()
        .map(|item| {
            let key = key_fn(&item);
            (key, item)
        })
        .collect();

    Ok(items_map)
}

/// Async version of query_array_to_map
pub async fn async_query_array_to_map<E, T, C, F>(
    endpoint: &E,
    client: &C,
    key_fn: F,
) -> Result<HashMap<String, T>, ApiError<C::Error>>
where
    E: Endpoint + Sync,
    T: DeserializeOwned + Send,
    C: AsyncClient + Sync,
    F: Fn(&T) -> String,
{
    let url = client.rest_endpoint(&endpoint.endpoint())?;

    let mut request = Request::builder()
        .method(endpoint.method())
        .uri(url.as_str());

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

    // Add body if present
    let body = if let Some((content_type, body_data)) = endpoint.body().map_err(ApiError::Body)? {
        request = request.header("Content-Type", content_type);
        body_data
    } else {
        Vec::new()
    };

    let request = request.body(body).map_err(|e| {
        ApiError::Server(e.to_string())
    })?;

    let response = client.rest(request).await?;

    let status = response.status();
    let headers = response.headers().clone();
    let body = response.into_body();

    if !status.is_success() {
        let error_msg = String::from_utf8_lossy(&body);
        let retry_after = crate::error::extract_retry_after(&headers);

        return match status.as_u16() {
            401 => Err(ApiError::Auth(format!("Unauthorized: {}", error_msg))),
            403 => Err(ApiError::Auth(format!("Forbidden: {}", error_msg))),
            400 => Err(ApiError::Server(format!("Bad Request (400): {}", error_msg))),
            404 => Err(ApiError::Server(format!("Not Found (404): {}", error_msg))),
            405 => Err(ApiError::Server(format!("Method Not Allowed (405): {}", error_msg))),
            406 => Err(ApiError::Server(format!("Not Acceptable (406): {}", error_msg))),
            409 => Err(ApiError::Server(format!("Conflict (409): {}", error_msg))),
            415 => Err(ApiError::Server(format!("Unsupported Media Type (415): {}", error_msg))),
            429 => {
                if let Some(seconds) = retry_after {
                    Err(ApiError::Server(format!("Too Many Requests (429): retry after {} seconds - {}", seconds, error_msg)))
                } else {
                    Err(ApiError::Server(format!("Too Many Requests (429): {}", error_msg)))
                }
            },
            499 => Err(ApiError::Server(format!("Timeout (499): {}", error_msg))),
            500 => Err(ApiError::Server(format!("Internal Server Error (500): {}", error_msg))),
            503 => {
                if let Some(seconds) = retry_after {
                    Err(ApiError::Server(format!("Service Unavailable (503): retry after {} seconds - {}", seconds, error_msg)))
                } else {
                    Err(ApiError::Server(format!("Service Unavailable (503): {}", error_msg)))
                }
            },
            307 => Err(ApiError::Server(format!("Temporary Redirect (307): {}", error_msg))),
            _ if (500..600).contains(&status.as_u16()) => Err(ApiError::Server(format!("Server Error ({}): {}", status, error_msg))),
            _ => Err(ApiError::Server(format!("HTTP Error ({}): {}", status, error_msg))),
        };
    }

    // Parse JSON array and convert to HashMap using the provided key function
    let items_array: Vec<T> = serde_json::from_slice(&body)
        .map_err(ApiError::data_type::<Vec<T>>)?;

    let items_map = items_array
        .into_iter()
        .map(|item| {
            let key = key_fn(&item);
            (key, item)
        })
        .collect();

    Ok(items_map)
}

/// Wrapper type that automatically converts API array responses to HashMaps
///
/// Usage: Query<ArrayToMap<Element>, C> instead of Query<HashMap<String, Element>, C>
#[derive(Debug)]
pub struct ArrayToMap<T>(pub HashMap<String, T>);

impl<T> ArrayToMap<T> {
    pub fn into_inner(self) -> HashMap<String, T> {
        self.0
    }
}

impl<T> std::ops::Deref for ArrayToMap<T> {
    type Target = HashMap<String, T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> std::ops::DerefMut for ArrayToMap<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T> From<ArrayToMap<T>> for HashMap<String, T> {
    fn from(wrapper: ArrayToMap<T>) -> Self {
        wrapper.0
    }
}

// Automatic deserialization for ArrayToMap when T implements HasKey
impl<'de, T> serde::Deserialize<'de> for ArrayToMap<T>
where
    T: serde::Deserialize<'de> + HasKey,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let items: Vec<T> = Vec::deserialize(deserializer)?;
        let map = items
            .into_iter()
            .map(|item| {
                let key = item.key();
                (key, item)
            })
            .collect();
        Ok(ArrayToMap(map))
    }
}
