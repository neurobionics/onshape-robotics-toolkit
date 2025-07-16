def generate_nonce() -> str: ...

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

class RootAssembly:
    full_configuration: str
    configuration: str
    document_id: str
    element_id: str
    document_microversion: str
    mass_property: MassProperties | None
    document_meta_data: DocumentMetaData | None
    @property
    def instances(self) -> list[str]: ...
    @property
    def patterns(self) -> list[str]: ...
    @property
    def features(self) -> list[str]: ...
    @property
    def occurrences(self) -> list[str]: ...

class Assembly:
    root_assembly: RootAssembly
    document: Document | None
    name: str | None
    @property
    def sub_assemblies(self) -> list[str]: ...
    @property
    def parts(self) -> list[str]: ...
    @property
    def part_studio_features(self) -> list[str]: ...

class TranslationJob:
    id: str
    request_state: str
    result_external_data_ids: list[str] | None

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
