use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Simple in-memory cache for API responses
#[derive(Debug, Clone)]
pub struct ResponseCache {
    cache: Arc<RwLock<HashMap<String, CacheEntry>>>,
    max_size: usize,
}

#[derive(Debug, Clone)]
struct CacheEntry {
    data: Vec<u8>,
    timestamp: std::time::Instant,
    hits: usize,
}

impl ResponseCache {
    /// Create a new response cache
    pub fn new(max_size: usize) -> Self {
        Self {
            cache: Arc::new(RwLock::new(HashMap::new())),
            max_size,
        }
    }

    /// Get an entry from the cache
    pub async fn get(&self, key: &str) -> Option<Vec<u8>> {
        let mut cache = self.cache.write().await;
        if let Some(entry) = cache.get_mut(key) {
            entry.hits += 1;
            Some(entry.data.clone())
        } else {
            None
        }
    }

    /// Put an entry into the cache
    pub async fn put(&self, key: String, data: Vec<u8>) {
        let mut cache = self.cache.write().await;

        // Simple LRU eviction if we're at capacity
        if cache.len() >= self.max_size {
            // Remove oldest entry
            if let Some((oldest_key, _)) = cache
                .iter()
                .min_by_key(|(_, entry)| entry.timestamp)
                .map(|(k, v)| (k.clone(), v.timestamp))
            {
                cache.remove(&oldest_key);
            }
        }

        cache.insert(key, CacheEntry {
            data,
            timestamp: std::time::Instant::now(),
            hits: 0,
        });
    }

    /// Get cache statistics
    pub async fn stats(&self) -> CacheStats {
        let cache = self.cache.read().await;
        let total_hits: usize = cache.values().map(|entry| entry.hits).sum();

        CacheStats {
            size: cache.len(),
            max_size: self.max_size,
            total_hits,
        }
    }

    /// Clear the cache
    pub async fn clear(&self) {
        let mut cache = self.cache.write().await;
        cache.clear();
    }
}

#[derive(Debug, Clone)]
pub struct CacheStats {
    pub size: usize,
    pub max_size: usize,
    pub total_hits: usize,
}

impl Default for ResponseCache {
    fn default() -> Self {
        Self::new(1000) // Default to 1000 entries
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_cache_basic_operations() {
        let cache = ResponseCache::new(3);

        // Test put and get
        cache.put("key1".to_string(), b"data1".to_vec()).await;
        assert_eq!(cache.get("key1").await, Some(b"data1".to_vec()));
        assert_eq!(cache.get("missing").await, None);

        // Test stats
        let stats = cache.stats().await;
        assert_eq!(stats.size, 1);
        assert_eq!(stats.total_hits, 1);
    }

    #[tokio::test]
    async fn test_cache_eviction() {
        let cache = ResponseCache::new(2);

        // Fill cache to capacity
        cache.put("key1".to_string(), b"data1".to_vec()).await;
        cache.put("key2".to_string(), b"data2".to_vec()).await;

        // Add one more - should evict oldest
        cache.put("key3".to_string(), b"data3".to_vec()).await;

        let stats = cache.stats().await;
        assert_eq!(stats.size, 2);
    }
}
