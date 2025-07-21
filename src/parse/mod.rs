pub mod async_fetcher;
pub mod cache;
pub mod geometry;
pub mod instances;
pub mod mates;
pub mod models;
pub mod high_level_api;

// Re-export key types and functions
pub use async_fetcher::{AsyncFetcher, FetchConfig, MassPropertiesRequest};
pub use instances::{
    InstanceTraverser, SubassemblyFetcher, PartFetcher, ParseConfig, TraversalResult,
    InstanceMap, OccurrenceMap, IdToNameMap, sanitize_name, get_occurrence_name, join_mate_occurrences
};
pub use models::*;

// Constants that match the Python implementation
pub const MATE_JOINER: &str = "_to_";
pub const SUBASSEMBLY_JOINER: &str = "_SUB_";
pub const CHILD: usize = 0;
pub const PARENT: usize = 1;
pub const RELATION_CHILD: usize = 1;
pub const RELATION_PARENT: usize = 0;
