"""Type stubs for the native Rust module."""

from typing import Any

from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyInstance,
    MateFeatureData,
    MateRelationFeatureData,
    Occurrence,
    Part,
    PartInstance,
    RootAssembly,
    SubAssembly,
)

# Constants
MATE_JOINER: str
SUBASSEMBLY_JOINER: str
CHILD: int
PARENT: int
RELATION_CHILD: int
RELATION_PARENT: int

# Configuration classes
class ParseConfig:
    max_depth: int
    configuration: str | None
    with_mass_properties: bool
    include_mate_features: bool
    include_mate_connectors: bool
    include_non_solids: bool
    log_response: bool

    def __init__(
        self,
        max_depth: int = 0,
        configuration: str | None = None,
        with_mass_properties: bool = True,
        include_mate_features: bool = True,
        include_mate_connectors: bool = True,
        include_non_solids: bool = False,
        log_response: bool = False,
    ) -> None: ...

class FetchConfig:
    max_concurrent_requests: int
    request_timeout_secs: int
    max_retries: int
    retry_base_delay_ms: int
    retry_max_delay_ms: int
    batch_size: int
    batch_delay_ms: int

    def __init__(
        self,
        max_concurrent_requests: int = 15,
        request_timeout_secs: int = 60,
        max_retries: int = 3,
        retry_base_delay_ms: int = 100,
        retry_max_delay_ms: int = 5000,
        batch_size: int = 30,
        batch_delay_ms: int = 50,
    ) -> None: ...

class ParseResult:
    instances: dict[str, Any]
    occurrences: dict[str, Any]
    id_to_name_map: dict[str, str]
    subassemblies: dict[str, Any]
    rigid_subassemblies: dict[str, Any]
    parts: dict[str, Any]
    performance_stats: dict[str, Any]

class RustAssemblyParser:
    def __init__(
        self,
        access_key: str,
        secret_key: str,
        base_url: str,
        parse_config: ParseConfig,
    ) -> None: ...
    def parse_assembly_async(self, assembly: Any, max_depth: int) -> ParseResult: ...
    def cleanup(self) -> None: ...

# Main parsing functions
def get_instances_rust(
    assembly: Assembly,
    max_depth: int | None = None,
) -> tuple[dict[str, PartInstance | AssemblyInstance], dict[str, Occurrence], dict[str, str]]: ...
def get_mates_and_relations_rust(
    assembly: Assembly,
    subassemblies: dict[str, SubAssembly],
    rigid_subassemblies: dict[str, RootAssembly],
    id_to_name_map: dict[str, str],
    parts: dict[str, Part],
) -> tuple[dict[str, MateFeatureData], dict[str, MateRelationFeatureData]]: ...
def get_performance_metrics() -> dict[str, Any]: ...

# Model classes (basic stubs)
class OnshapeClient:
    def __init__(self, env_file_path: str | None = None, base_url: str | None = None) -> None: ...
    @property
    def base_url(self) -> str: ...
    @base_url.setter
    def base_url(self, base_url: str) -> None: ...
    @property
    def api_call_count(self) -> int: ...
    def get_elements(self, did: str, wtype: str, wid: str) -> dict[str, Element]: ...
    def get_document_metadata(self, did: str) -> DocumentMetaData: ...
    def get_variables(self, did: str, wid: str, eid: str) -> dict[str, Variable]: ...
    def set_variables(self, did: str, wid: str, eid: str, variables: dict[str, str]) -> bool: ...
    def get_assembly(self, did: str, wtype: str, wid: str, eid: str, configuration: str | None = None) -> Assembly: ...
    def get_root_assembly(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        configuration: str | None = None,
        with_mass_properties: bool | None = None,
    ) -> RootAssembly: ...
    def get_assembly_mass_properties(self, did: str, wtype: str, wid: str, eid: str) -> MassProperties: ...
    def get_mass_property(self, did: str, wtype: str, wid: str, eid: str, part_id: str) -> MassProperties: ...
    def download_part_stl(self, did: str, wtype: str, wid: str, eid: str, part_id: str) -> bytes: ...

class DocumentMetaData:
    id: str
    name: str
    default_workspace: DefaultWorkspace

class DefaultWorkspace:
    id: str
    workspace_type: str

class Element:
    id: str
    name: str
    element_type: str
    microversion_id: str

class Variable:
    var_type: str
    name: str
    description: str | None
    expression: str
    @property
    def value(self) -> str | None: ...

class Document:
    did: str
    wtype: str
    wid: str
    eid: str
    name: str | None
    url: str | None

class MassProperties:
    volume: list[float]
    mass: list[float]
    centroid: list[float]
    inertia: list[float]
    principal_inertia: list[float]
    principal_axes: list[float]

# Assembly and RootAssembly are imported from models.assembly

class TranslationJob:
    id: str
    request_state: str
    result_external_data_ids: list[str] | None
