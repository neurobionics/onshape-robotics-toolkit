use std::error::Error;
use std::fmt;

#[derive(Debug)]
pub enum OnshapeError {
    Http(reqwest::Error),
    Auth(String),
    InvalidInput(String),
    NotFound(String),
    Forbidden(String),
    TooManyRequests { retry_after: Option<u64> },
    Server(String),
    Serialization(serde_json::Error),
    Io(std::io::Error),
}

impl fmt::Display for OnshapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OnshapeError::Http(e) => write!(f, "HTTP error: {}", e),
            OnshapeError::Auth(msg) => write!(f, "Authentication error: {}", msg),
            OnshapeError::InvalidInput(msg) => write!(f, "Invalid input: {}", msg),
            OnshapeError::NotFound(msg) => write!(f, "Not found: {}", msg),
            OnshapeError::Forbidden(msg) => write!(f, "Forbidden: {}", msg),
            OnshapeError::TooManyRequests { retry_after } => {
                if let Some(seconds) = retry_after {
                    write!(f, "Too many requests, retry after {} seconds", seconds)
                } else {
                    write!(f, "Too many requests")
                }
            }
            OnshapeError::Server(msg) => write!(f, "Server error: {}", msg),
            OnshapeError::Serialization(e) => write!(f, "Serialization error: {}", e),
            OnshapeError::Io(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl Error for OnshapeError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            OnshapeError::Http(e) => Some(e),
            OnshapeError::Serialization(e) => Some(e),
            OnshapeError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<reqwest::Error> for OnshapeError {
    fn from(error: reqwest::Error) -> Self {
        OnshapeError::Http(error)
    }
}

impl From<serde_json::Error> for OnshapeError {
    fn from(error: serde_json::Error) -> Self {
        OnshapeError::Serialization(error)
    }
}

impl From<std::io::Error> for OnshapeError {
    fn from(error: std::io::Error) -> Self {
        OnshapeError::Io(error)
    }
}

#[derive(Debug)]
pub enum ApiError<E> {
    Client(E),
    Auth(String),
    DataType {
        source_type: String,
        target_type: String,
        source: serde_json::Error,
    },
    Server(String),
    Body(BodyError),
}

impl<E> fmt::Display for ApiError<E>
where
    E: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ApiError::Client(e) => write!(f, "Client error: {}", e),
            ApiError::Auth(msg) => write!(f, "Authentication error: {}", msg),
            ApiError::DataType {
                source_type,
                target_type,
                source,
            } => write!(
                f,
                "Failed to deserialize {} into {}: {}",
                source_type, target_type, source
            ),
            ApiError::Server(msg) => write!(f, "Server error: {}", msg),
            ApiError::Body(e) => write!(f, "Body error: {}", e),
        }
    }
}

impl<E> Error for ApiError<E>
where
    E: Error + 'static,
{
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            ApiError::Client(e) => Some(e),
            ApiError::DataType { source, .. } => Some(source),
            ApiError::Body(e) => Some(e),
            _ => None,
        }
    }
}

impl<E> ApiError<E> {
    pub fn data_type<T>(source: serde_json::Error) -> Self {
        ApiError::DataType {
            source_type: "JSON".to_string(),
            target_type: std::any::type_name::<T>().to_string(),
            source,
        }
    }
}

#[derive(Debug)]
pub enum BodyError {
    Serialization(serde_json::Error),
    Encoding(std::str::Utf8Error),
}

impl fmt::Display for BodyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BodyError::Serialization(e) => write!(f, "Body serialization error: {}", e),
            BodyError::Encoding(e) => write!(f, "Body encoding error: {}", e),
        }
    }
}

impl Error for BodyError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            BodyError::Serialization(e) => Some(e),
            BodyError::Encoding(e) => Some(e),
        }
    }
}

impl From<serde_json::Error> for BodyError {
    fn from(error: serde_json::Error) -> Self {
        BodyError::Serialization(error)
    }
}

impl From<std::str::Utf8Error> for BodyError {
    fn from(error: std::str::Utf8Error) -> Self {
        BodyError::Encoding(error)
    }
}
