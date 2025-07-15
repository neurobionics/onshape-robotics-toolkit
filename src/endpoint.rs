use reqwest::Method;

pub trait Endpoint {
    fn method(&self) -> Method;
    fn path(&self) -> String;
    fn query(&self) -> HashMap<String, String>;
    fn body(&self) -> HashMap<String, String>;
}
