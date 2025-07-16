pub mod models;
pub mod instances;
pub mod mates;
pub mod geometry;
pub mod async_fetcher;

// Re-export public API
pub use models::*;
pub use instances::*;
pub use mates::*;
pub use geometry::*;
pub use async_fetcher::*;

// Constants from Python parse.py
pub const SUBASSEMBLY_JOINER: &str = "_SUB_";
pub const MATE_JOINER: &str = "_to_";
pub const CHILD: usize = 0;
pub const PARENT: usize = 1;
pub const RELATION_CHILD: usize = 1;
pub const RELATION_PARENT: usize = 0;
