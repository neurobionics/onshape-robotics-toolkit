use pyo3::prelude::*;

#[pyclass]
pub struct OnshapeClient {
    access_key: String,
    secret_key: String,
    base_url: String,
    count: u32,
}

#[pymethods]
impl OnshapeClient {
    #[new]
    pub fn new(access_key: String, secret_key: String, base_url: String) -> Self {
        Self { access_key, secret_key, base_url, count: 0 }
    }

    #[getter]
    pub fn get_count(&self) -> u32 {
        self.count
    }

    #[setter]
    pub fn set_count(&mut self, count: u32) {
        self.count = count;
    }

    #[getter]
    pub fn get_base_url(&self) -> String {
        self.base_url.clone()
    }

    #[setter]
    pub fn set_base_url(&mut self, base_url: String) {
        self.base_url = base_url;
    }

    pub fn increment_count(&mut self) -> u32 {
        self.count += 1;
        self.count
    }

    pub fn reset_count(&mut self) {
        self.count = 0;
    }
}
