use std::collections::HashMap;
use url::form_urlencoded;

use crate::error::BodyError;

/// Helper for building query parameters
#[derive(Debug, Default, Clone)]
pub struct QueryParams {
    params: HashMap<String, String>,
}

impl QueryParams {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a parameter
    pub fn push<K, V>(&mut self, key: K, value: V) -> &mut Self
    where
        K: ToString,
        V: ToString,
    {
        self.params.insert(key.to_string(), value.to_string());
        self
    }

    /// Add an optional parameter
    pub fn push_opt<K, V>(&mut self, key: K, value: Option<V>) -> &mut Self
    where
        K: ToString,
        V: ToString,
    {
        if let Some(v) = value {
            self.push(key, v);
        }
        self
    }

    /// Add a boolean parameter (only if true)
    pub fn push_bool<K>(&mut self, key: K, value: bool) -> &mut Self
    where
        K: ToString,
    {
        if value {
            self.push(key, "true");
        }
        self
    }

    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.params.iter().map(|(k, v)| (k.as_str(), v.as_str()))
    }
}

/// Helper for building form-encoded request bodies
#[derive(Debug, Default)]
pub struct FormParams {
    params: Vec<(String, String)>,
}

impl FormParams {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a parameter
    pub fn push<K, V>(&mut self, key: K, value: V) -> &mut Self
    where
        K: ToString,
        V: ToString,
    {
        self.params.push((key.to_string(), value.to_string()));
        self
    }

    /// Add an optional parameter
    pub fn push_opt<K, V>(&mut self, key: K, value: Option<V>) -> &mut Self
    where
        K: ToString,
        V: ToString,
    {
        if let Some(v) = value {
            self.push(key, v);
        }
        self
    }

    /// Add a boolean parameter (only if true)
    pub fn push_bool<K>(&mut self, key: K, value: bool) -> &mut Self
    where
        K: ToString,
    {
        if value {
            self.push(key, "true");
        }
        self
    }

    /// Convert to a URL-encoded body
    pub fn into_body(self) -> Result<Option<(&'static str, Vec<u8>)>, BodyError> {
        if self.params.is_empty() {
            return Ok(None);
        }

        let encoded = form_urlencoded::Serializer::new(String::new())
            .extend_pairs(self.params)
            .finish();

        Ok(Some((
            "application/x-www-form-urlencoded",
            encoded.into_bytes(),
        )))
    }
}
