use std::borrow::Cow;

use derive_builder::Builder;
use http::Method;

use crate::endpoint::{Endpoint, ArrayToMap};
use crate::error::BodyError;
use crate::params::QueryParams;
use crate::model::Element;

/// Get document metadata
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetDocumentMetadata<'a> {
    /// Document ID
    pub did: &'a str,
}

impl<'a> Endpoint for GetDocumentMetadata<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!("/api/documents/{}", self.did).into()
    }
}

/// Get all elements in a document
#[derive(Debug, Clone)]
pub struct GetElements<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
}

impl<'a> Endpoint for GetElements<'a> {
    fn method(&self) -> http::Method {
        http::Method::GET
    }

    fn endpoint(&self) -> std::borrow::Cow<'static, str> {
        format!("/api/documents/d/{}/{}/{}/elements", self.did, self.wtype, self.wid).into()
    }
}

// Type alias for convenience - this is what the get_elements method should return
pub type ElementsMap = ArrayToMap<Element>;

/// Get variables from a variable studio
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetVariables<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (variable studio)
    pub eid: &'a str,
}

impl<'a> Endpoint for GetVariables<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/variables/d/{}/w/{}/e/{}/variables",
            self.did, self.wid, self.eid
        )
        .into()
    }
}

/// Set variables in a variable studio
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct SetVariables<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (variable studio)
    pub eid: &'a str,
    /// Variables to set (name -> expression)
    pub variables: Vec<VariableUpdate<'a>>,
}

#[derive(Debug, Clone)]
pub struct VariableUpdate<'a> {
    pub name: &'a str,
    pub expression: &'a str,
}

impl<'a> Endpoint for SetVariables<'a> {
    fn method(&self) -> Method {
        Method::POST
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/variables/d/{}/w/{}/e/{}/variables",
            self.did, self.wid, self.eid
        )
        .into()
    }

    fn body(&self) -> Result<Option<(&'static str, Vec<u8>)>, BodyError> {
        let payload: Vec<serde_json::Value> = self
            .variables
            .iter()
            .map(|var| {
                serde_json::json!({
                    "name": var.name,
                    "expression": var.expression
                })
            })
            .collect();

        let body = serde_json::to_vec(&payload)?;
        Ok(Some(("application/json", body)))
    }
}

/// Get assembly data
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetAssembly<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (assembly)
    pub eid: &'a str,
    /// Configuration string
    #[builder(default = "\"default\"")]
    pub configuration: &'a str,
    /// Include mate features
    #[builder(default = "true")]
    pub include_mate_features: bool,
    /// Include mate connectors
    #[builder(default = "true")]
    pub include_mate_connectors: bool,
    /// Include non-solids
    #[builder(default = "false")]
    pub include_non_solids: bool,
}

impl<'a> Endpoint for GetAssembly<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/assemblies/d/{}/{}/{}/e/{}",
            self.did, self.wtype, self.wid, self.eid
        )
        .into()
    }

    fn parameters(&self) -> QueryParams {
        let mut params = QueryParams::new();
        params
            .push_bool("includeMateFeatures", self.include_mate_features)
            .push_bool("includeMateConnectors", self.include_mate_connectors)
            .push_bool("includeNonSolids", self.include_non_solids)
            .push("configuration", self.configuration);
        params
    }
}

/// Download STL from a part studio
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct DownloadPartStl<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (part studio)
    pub eid: &'a str,
    /// Part ID
    pub part_id: &'a str,
    /// Mode (binary or text)
    #[builder(default = "\"binary\"")]
    pub mode: &'a str,
    /// Grouping
    #[builder(default = "true")]
    pub grouping: bool,
    /// Units
    #[builder(default = "\"meter\"")]
    pub units: &'a str,
}

impl<'a> Endpoint for DownloadPartStl<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/parts/d/{}/{}/{}/e/{}/partid/{}/stl",
            self.did, self.wtype, self.wid, self.eid, self.part_id
        )
        .into()
    }

    fn parameters(&self) -> QueryParams {
        let mut params = QueryParams::new();
        params
            .push("mode", self.mode)
            .push_bool("grouping", self.grouping)
            .push("units", self.units);
        params
    }
}

/// Get mass properties for a part
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetPartMassProperties<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (part studio)
    pub eid: &'a str,
    /// Part ID
    pub part_id: &'a str,
    /// Use mass properties overrides
    #[builder(default = "true")]
    pub use_mass_properties_overrides: bool,
}

impl<'a> Endpoint for GetPartMassProperties<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/parts/d/{}/{}/{}/e/{}/partid/{}/massproperties",
            self.did, self.wtype, self.wid, self.eid, self.part_id
        )
        .into()
    }

    fn parameters(&self) -> QueryParams {
        let mut params = QueryParams::new();
        params.push_bool("useMassPropertiesOverrides", self.use_mass_properties_overrides);
        params
    }
}

/// Get assembly mass properties
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetAssemblyMassProperties<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (assembly)
    pub eid: &'a str,
}

impl<'a> Endpoint for GetAssemblyMassProperties<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/assemblies/d/{}/{}/{}/e/{}/massproperties",
            self.did, self.wtype, self.wid, self.eid
        )
        .into()
    }
}

/// Initiate STL translation for an assembly
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct CreateAssemblyTranslation<'a> {
    /// Document ID
    pub did: &'a str,
    /// Workspace type (w, v, or m)
    pub wtype: &'a str,
    /// Workspace ID
    pub wid: &'a str,
    /// Element ID (assembly)
    pub eid: &'a str,
    /// Configuration string
    #[builder(default = "\"default\"")]
    pub configuration: &'a str,
}

impl<'a> Endpoint for CreateAssemblyTranslation<'a> {
    fn method(&self) -> Method {
        Method::POST
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!(
            "/api/assemblies/d/{}/{}/{}/e/{}/translations",
            self.did, self.wtype, self.wid, self.eid
        )
        .into()
    }

    fn body(&self) -> Result<Option<(&'static str, Vec<u8>)>, BodyError> {
        let payload = serde_json::json!({
            "formatName": "STL",
            "storeInDocument": "false",
            "configuration": self.configuration
        });

        let body = serde_json::to_vec(&payload)?;
        Ok(Some(("application/json", body)))
    }
}

/// Get translation status
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct GetTranslationStatus<'a> {
    /// Translation ID
    pub translation_id: &'a str,
}

impl<'a> Endpoint for GetTranslationStatus<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!("/api/translations/{}", self.translation_id).into()
    }
}

/// Download external data (for completed translations)
#[derive(Debug, Builder)]
#[builder(setter(into))]
pub struct DownloadExternalData<'a> {
    /// Document ID
    pub did: &'a str,
    /// External data ID
    pub fid: &'a str,
}

impl<'a> Endpoint for DownloadExternalData<'a> {
    fn method(&self) -> Method {
        Method::GET
    }

    fn endpoint(&self) -> Cow<'static, str> {
        format!("/api/documents/d/{}/externaldata/{}", self.did, self.fid).into()
    }
}
