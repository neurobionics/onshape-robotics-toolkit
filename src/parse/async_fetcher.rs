use std::collections::HashMap;
use std::time::Duration;
use futures::future::join_all;
use tokio::time::sleep;
use crate::client::AsyncOnshapeClient;
use crate::endpoint::AsyncQuery;
use crate::endpoints::{GetPartMassProperties, GetAssemblyMassProperties};
use crate::model::MassProperties;

/// Configuration for async fetching behavior
#[derive(Debug, Clone)]
pub struct FetchConfig {
    pub max_concurrent_requests: usize,
    pub request_timeout_secs: u64,
    pub max_retries: usize,
    pub retry_base_delay_ms: u64,
    pub retry_max_delay_ms: u64,
    pub batch_size: usize,
    pub batch_delay_ms: u64,
}

impl Default for FetchConfig {
    fn default() -> Self {
        Self {
            max_concurrent_requests: 15,
            request_timeout_secs: 60,
            max_retries: 3,
            retry_base_delay_ms: 100,
            retry_max_delay_ms: 5000,
            batch_size: 30,
            batch_delay_ms: 50,
        }
    }
}

/// Request for mass properties (either part or assembly)
#[derive(Debug, Clone)]
pub enum MassPropertiesRequest {
    Part {
        document_id: String,
        document_microversion: String,
        element_id: String,
        part_id: String,
    },
    Assembly {
        document_id: String,
        document_microversion: String,
        element_id: String,
    },
}

impl MassPropertiesRequest {
    pub fn part(
        document_id: String,
        document_microversion: String,
        element_id: String,
        part_id: String,
    ) -> Self {
        Self::Part {
            document_id,
            document_microversion,
            element_id,
            part_id,
        }
    }

    pub fn assembly(
        document_id: String,
        document_microversion: String,
        element_id: String,
    ) -> Self {
        Self::Assembly {
            document_id,
            document_microversion,
            element_id,
        }
    }

    /// Generate a unique key for this request for result mapping
    pub fn key(&self) -> String {
        match self {
            Self::Part { document_id, document_microversion, element_id, part_id } => {
                format!("{}:{}:{}:{}", document_id, document_microversion, element_id, part_id)
            }
            Self::Assembly { document_id, document_microversion, element_id } => {
                format!("{}:{}:{}", document_id, document_microversion, element_id)
            }
        }
    }
}

/// Async fetcher for concurrent API calls with intelligent batching and retries
pub struct AsyncFetcher {
    config: FetchConfig,
}

impl AsyncFetcher {
    pub fn new(config: FetchConfig) -> Self {
        Self { config }
    }

    /// Fetch mass properties in optimized batches with concurrency control
    pub async fn fetch_mass_properties_batch(
        &self,
        client: &AsyncOnshapeClient,
        requests: Vec<MassPropertiesRequest>,
    ) -> Result<HashMap<String, MassProperties>, String> {
        if requests.is_empty() {
            return Ok(HashMap::new());
        }

        log::info!("Starting batch fetch of {} mass properties requests", requests.len());

        let mut all_results = HashMap::new();
        let total_batches = (requests.len() + self.config.batch_size - 1) / self.config.batch_size;

        // Process requests in batches to manage memory and rate limiting
        for (batch_index, batch) in requests.chunks(self.config.batch_size).enumerate() {
            log::debug!("Processing batch {}/{} with {} requests", batch_index + 1, total_batches, batch.len());

            // Add delay between batches to avoid overwhelming the API
            if batch_index > 0 {
                sleep(Duration::from_millis(self.config.batch_delay_ms)).await;
            }

            let batch_results = self.fetch_batch_concurrent(client, batch).await?;
            all_results.extend(batch_results);

            log::debug!("Batch {}/{} completed, {} total results", batch_index + 1, total_batches, all_results.len());
        }

        log::info!("Batch fetch completed: {} successful results from {} requests",
                  all_results.len(), requests.len());
        Ok(all_results)
    }

    /// Fetch a single batch with controlled concurrency
    async fn fetch_batch_concurrent(
        &self,
        client: &AsyncOnshapeClient,
        batch: &[MassPropertiesRequest],
    ) -> Result<HashMap<String, MassProperties>, String> {
        // Split batch into concurrent chunks
        let chunk_size = std::cmp::max(1, self.config.max_concurrent_requests);
        let mut results = HashMap::new();

        for chunk in batch.chunks(chunk_size) {
            let futures: Vec<_> = chunk.iter().map(|request| {
                self.fetch_single_with_retry(client, request.clone())
            }).collect();

            let chunk_results = join_all(futures).await;

            // Collect successful results
            for result in chunk_results {
                match result {
                    Ok((key, mass_props)) => {
                        results.insert(key, mass_props);
                    }
                    Err(e) => {
                        log::warn!("Failed to fetch mass properties: {}", e);
                        // Continue with other requests rather than failing the entire batch
                    }
                }
            }
        }

        Ok(results)
    }

    /// Fetch a single mass properties request with exponential backoff retry
    async fn fetch_single_with_retry(
        &self,
        client: &AsyncOnshapeClient,
        request: MassPropertiesRequest,
    ) -> Result<(String, MassProperties), String> {
        let key = request.key();
        let mut last_error = String::new();

        for attempt in 0..=self.config.max_retries {
            if attempt > 0 {
                // Exponential backoff with jitter
                let delay_ms = std::cmp::min(
                    self.config.retry_base_delay_ms * (2_u64.pow(attempt as u32 - 1)),
                    self.config.retry_max_delay_ms,
                );

                log::debug!("Retrying request {} (attempt {}/{}) after {}ms",
                           key, attempt + 1, self.config.max_retries + 1, delay_ms);
                sleep(Duration::from_millis(delay_ms)).await;
            }

            match self.fetch_single(client, &request).await {
                Ok(mass_props) => {
                    if attempt > 0 {
                        log::debug!("Request {} succeeded on attempt {}/{}",
                                   key, attempt + 1, self.config.max_retries + 1);
                    }
                    return Ok((key, mass_props));
                }
                Err(e) => {
                    last_error = e;
                    log::debug!("Request {} failed on attempt {}/{}: {}",
                               key, attempt + 1, self.config.max_retries + 1, last_error);
                }
            }
        }

        Err(format!("Failed to fetch {} after {} attempts: {}", key, self.config.max_retries + 1, last_error))
    }

    /// Fetch a single mass properties request
    async fn fetch_single(
        &self,
        client: &AsyncOnshapeClient,
        request: &MassPropertiesRequest,
    ) -> Result<MassProperties, String> {
        match request {
            MassPropertiesRequest::Part { document_id, document_microversion, element_id, part_id } => {
                let endpoint = GetPartMassProperties {
                    did: document_id,
                    wtype: "m",
                    wid: document_microversion,
                    eid: element_id,
                    part_id,
                    use_mass_properties_overrides: true,
                };

                AsyncQuery::query(&endpoint, client).await
                    .map_err(|e| format!("Failed to fetch part mass properties: {}", e))
            }
            MassPropertiesRequest::Assembly { document_id, document_microversion, element_id } => {
                let endpoint = GetAssemblyMassProperties {
                    did: document_id,
                    wtype: "m",
                    wid: document_microversion,
                    eid: element_id,
                };

                AsyncQuery::query(&endpoint, client).await
                    .map_err(|e| format!("Failed to fetch assembly mass properties: {}", e))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fetch_config_default() {
        let config = FetchConfig::default();
        assert_eq!(config.max_concurrent_requests, 15);
        assert_eq!(config.batch_size, 30);
        assert_eq!(config.max_retries, 3);
    }

    #[test]
    fn test_mass_properties_request_key() {
        let part_request = MassPropertiesRequest::part(
            "doc1".to_string(),
            "mv1".to_string(),
            "elem1".to_string(),
            "part1".to_string(),
        );
        assert_eq!(part_request.key(), "doc1:mv1:elem1:part1");

        let assembly_request = MassPropertiesRequest::assembly(
            "doc2".to_string(),
            "mv2".to_string(),
            "elem2".to_string(),
        );
        assert_eq!(assembly_request.key(), "doc2:mv2:elem2");
    }

    #[test]
    fn test_async_fetcher_creation() {
        let config = FetchConfig::default();
        let fetcher = AsyncFetcher::new(config);
        assert_eq!(fetcher.config.max_concurrent_requests, 15);
    }
}
