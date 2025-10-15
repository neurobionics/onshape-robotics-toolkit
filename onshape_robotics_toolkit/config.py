"""
Configuration data models and session recording utilities for the toolkit.

The goal of this module is to provide a lightweight way to capture every
parameter involved in the CAD → Robot conversion pipeline so that users can
persist them to YAML (and load them later) regardless of whether they use the
high-level CLI or interact with the library directly.

Unlike previous revisions, the configuration objects defined here do not
execute the pipeline. They simply store data. To capture parameters
automatically, wrap your script with :func:`record_session`.

Example
-------
>>> from onshape_robotics_toolkit.config import record_session
>>> with record_session(save_path="output/session.yaml"):
...     client = Client(env=".env")
...     doc = Document.from_url("https://cad.onshape.com/documents/...")
...     assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)
...     cad = CAD.from_assembly(assembly, max_depth=2, client=client)
...     graph = KinematicGraph.from_cad(cad)
...     robot = Robot.from_graph(graph, client=client, name="demo")
...     robot.save("output/demo.urdf")
"""

from __future__ import annotations

import atexit
from collections.abc import Generator, Mapping, Set
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any, Literal, cast

import yaml
from loguru import logger
from pydantic import BaseModel, Field, field_validator, model_validator

from onshape_robotics_toolkit.models.document import BASE_URL, generate_url, parse_url

__all__ = [
    "AssemblyConfig",
    "CADConfig",
    "ClientConfig",
    "DocumentConfig",
    "ExportConfig",
    "KinematicsConfig",
    "LoggingConfig",
    "ORTConfig",
    "ORTSession",
    "PreprocessingConfig",
    "RobotBuildConfig",
    "VariableUpdateConfig",
    "activate_config",
    "configure_auto_save",
    "get_active_session",
    "record_assembly_config",
    "record_cad_config",
    "record_client_config",
    "record_document_config",
    "record_export_config",
    "record_kinematics_config",
    "record_logging_config",
    "record_mate_name",
    "record_part_name",
    "record_robot_config",
    "record_session",
    "record_variable_update",
    "resolve_mate_limits",
    "resolve_mate_name",
    "resolve_part_name",
    "save_active_session",
    "update_mate_limits",
]


class LoggingConfig(BaseModel):
    """Captures how logging was configured for a run."""

    mode: Literal["default", "minimal", "quiet"] = Field(default="default")
    console_level: str = Field(default="INFO")
    file_level: str = Field(default="DEBUG")
    file_path: str | None = Field(default="onshape_toolkit.log")
    clear_existing_handlers: bool = Field(default=True)
    delay_file_creation: bool = Field(default=False)


class ClientConfig(BaseModel):
    """Parameters supplied when constructing :class:`Client`."""

    env: str | None = Field(default=None)
    base_url: str = Field(default=BASE_URL)


class DocumentConfig(BaseModel):
    """Document identifiers collected from URLs or Onshape API responses."""

    url: str | None = Field(default=None)
    base_url: str | None = Field(default=None)
    did: str | None = Field(default=None)
    wtype: str | None = Field(default=None)
    wid: str | None = Field(default=None)
    eid: str | None = Field(default=None)
    name: str | None = Field(default=None)

    @model_validator(mode="after")
    def _validate_source(self) -> DocumentConfig:
        if self.url is not None:
            return self

        required = ("base_url", "did", "wtype", "wid", "eid")
        missing = [field for field in required if getattr(self, field) is None]
        if missing:
            raise ValueError(
                "Document configuration requires either 'url' or all of "
                "'base_url', 'did', 'wtype', 'wid', and 'eid'. Missing: " + ", ".join(missing)
            )
        return self

    @classmethod
    def from_url(cls, url: str, *, name: str | None = None) -> DocumentConfig:
        base_url, did, wtype, wid, eid = parse_url(url)
        return cls(url=url, base_url=base_url, did=did, wtype=wtype, wid=wid, eid=eid, name=name)

    def as_url(self) -> str:
        """Return a canonical document URL."""
        if self.url:
            return self.url
        base_url = self.base_url or BASE_URL
        return generate_url(
            base_url=base_url,
            did=self.did or "",
            wtype=self.wtype or "",
            wid=self.wid or "",
            eid=self.eid or "",
        )


class VariableUpdateConfig(BaseModel):
    """Represents a batch of variable studio updates."""

    element_id: str = Field(..., description="Variable studio element ID.")
    expressions: dict[str, str] = Field(default_factory=dict)

    @field_validator("expressions", mode="before")
    @classmethod
    def _normalize(cls, value: Any) -> dict[str, str]:
        if isinstance(value, dict):
            return {str(key): str(val) for key, val in value.items()}
        raise TypeError("expressions must be a mapping")


class PreprocessingConfig(BaseModel):
    """Collects any preprocessing steps executed before fetching the assembly."""

    variable_updates: list[VariableUpdateConfig] = Field(default_factory=list)


class NameOverrideEntry(BaseModel):
    """Stores mapping information for a single entity name."""

    original: str | None = Field(default=None, description="Original name as provided by Onshape.")
    name: str = Field(default="", description="Preferred name when exporting or displaying.")
    limits: dict[str, float] | None = Field(
        default=None, description="Joint limits for mates {'min': lower_limit, 'max': upper_limit}."
    )


class NameOverrides(BaseModel):
    """Container for name overrides keyed by sanitized identifiers."""

    parts: dict[str, NameOverrideEntry] = Field(default_factory=dict)
    mates: dict[str, NameOverrideEntry] = Field(default_factory=dict)

    def copy(
        self,
        *,
        include: Set[int] | Set[str] | Mapping[int, Any] | Mapping[str, Any] | None = None,
        exclude: Set[int] | Set[str] | Mapping[int, Any] | Mapping[str, Any] | None = None,
        update: dict[str, Any] | None = None,
        deep: bool = False,
    ) -> NameOverrides:
        return cast(
            "NameOverrides",
            super().copy(include=include, exclude=exclude, update=update, deep=deep),
        )


class AssemblyConfig(BaseModel):
    """Parameters passed to :meth:`Client.get_assembly`."""

    element_id: str | None = Field(default=None)
    configuration: str = Field(default="default")
    log_response: bool = Field(default=True)
    with_meta_data: bool = Field(default=True)


class CADConfig(BaseModel):
    """Parameters passed to :meth:`CAD.from_assembly`."""

    max_depth: int = Field(default=0)


class KinematicsConfig(BaseModel):
    """Parameters used when constructing :class:`KinematicGraph`."""

    use_user_defined_root: bool = Field(default=True)


class RobotBuildConfig(BaseModel):
    """Parameters supplied to :meth:`Robot.from_graph`."""

    name: str = Field(...)
    type: Literal["urdf", "xml"] = Field(default="urdf")
    fetch_mass_properties: bool = Field(default=True)


class ExportConfig(BaseModel):
    """Parameters supplied to :meth:`Robot.save`."""

    file_path: str | None = Field(default=None)
    download_assets: bool = Field(default=True)
    mesh_dir: str | None = Field(default=None)


class ORTConfig(BaseModel):
    """Aggregated configuration covering the entire workflow."""

    logging: LoggingConfig | None = None
    client: ClientConfig | None = None
    document: DocumentConfig | None = None
    preprocessing: PreprocessingConfig | None = None
    assembly: AssemblyConfig | None = None
    cad: CADConfig | None = None
    kinematics: KinematicsConfig | None = None
    robot: RobotBuildConfig | None = None
    export: ExportConfig | None = None
    names: NameOverrides = Field(default_factory=NameOverrides)

    def save(self, path: Path | str) -> None:
        path_obj = Path(path)
        path_obj.parent.mkdir(parents=True, exist_ok=True)
        with path_obj.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(self.model_dump(mode="json", exclude_none=True), handle, sort_keys=False)

    @classmethod
    def load(cls, path: Path | str) -> ORTConfig:
        with Path(path).open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        config = cls.model_validate(data)
        configure_auto_save(path)
        activate_config(config, reset=True)
        return config


class ORTSession:
    """Accumulates configuration data during a run."""

    def __init__(self) -> None:
        self.logging: LoggingConfig | None = None
        self.client: ClientConfig | None = None
        self.document: DocumentConfig | None = None
        self.preprocessing: PreprocessingConfig | None = None
        self.assembly: AssemblyConfig | None = None
        self.cad: CADConfig | None = None
        self.kinematics: KinematicsConfig | None = None
        self.robot: RobotBuildConfig | None = None
        self.export: ExportConfig | None = None
        self.names: NameOverrides = NameOverrides()
        self.auto_save_path: Path | None = None

    def to_config(self) -> ORTConfig:
        return ORTConfig(
            logging=self.logging,
            client=self.client,
            document=self.document,
            preprocessing=self.preprocessing,
            assembly=self.assembly,
            cad=self.cad,
            kinematics=self.kinematics,
            robot=self.robot,
            export=self.export,
            names=self.names.copy(),
        )

    def save(self, path: Path | str) -> None:
        logger.info(f"Saving session configuration to {path}")
        self.to_config().save(path)

    def reset(self) -> None:
        self.logging = None
        self.client = None
        self.document = None
        self.preprocessing = None
        self.assembly = None
        self.cad = None
        self.kinematics = None
        self.robot = None
        self.export = None
        self.names = NameOverrides()
        self.auto_save_path = None

    def merge_from(self, other: ORTSession) -> None:
        for field in (
            "logging",
            "client",
            "document",
            "preprocessing",
            "assembly",
            "cad",
            "kinematics",
            "robot",
            "export",
        ):
            value = getattr(other, field)
            if value is not None:
                setattr(self, field, value)
        if other.names.parts or other.names.mates:
            self.names = other.names.copy()
        if other.auto_save_path is not None:
            self.auto_save_path = other.auto_save_path


_GLOBAL_SESSION = ORTSession()
_ACTIVE_SESSION: ContextVar[ORTSession] = ContextVar("_ACTIVE_SESSION", default=_GLOBAL_SESSION)


def get_active_session() -> ORTSession:
    """Return the currently active session recorder."""
    return _ACTIVE_SESSION.get()


def save_active_session(path: Path | str) -> None:
    """Persist the currently active session to disk."""
    session = get_active_session()
    path_obj = Path(path)
    session.auto_save_path = path_obj
    session.save(path_obj)


def configure_auto_save(path: Path | str) -> None:
    """Configure the destination used for automatic session persistence on exit."""

    session = get_active_session()
    session.auto_save_path = Path(path)


@contextmanager
def record_session(*, save_path: Path | str | None = None) -> Generator[ORTSession, None, None]:
    """
    Context manager that captures configuration data produced during execution.

    Args:
        save_path: Optional path to write the resulting YAML when the context exits.
    """
    parent_session = get_active_session()
    session = ORTSession()
    session.names = parent_session.names.copy()
    session.auto_save_path = parent_session.auto_save_path
    token = _ACTIVE_SESSION.set(session)
    try:
        yield session
    finally:
        if save_path is not None:
            session.auto_save_path = Path(save_path)
            session.save(save_path)
        parent_session.merge_from(session)
        _ACTIVE_SESSION.reset(token)


def activate_config(config: ORTConfig, *, reset: bool = False) -> None:
    """Apply persisted configuration (currently name overrides) to the active session."""

    session = get_active_session()
    previous_auto_path = session.auto_save_path
    if reset:
        session.reset()
        session.auto_save_path = previous_auto_path
    session.names = config.names.copy()


def _update_session(field: str, value: Any) -> None:
    if value is None:
        return
    session = get_active_session()
    setattr(session, field, value)


def record_logging_config(config: LoggingConfig) -> None:
    _update_session("logging", config)


def record_client_config(env: str | None, base_url: str) -> None:
    _update_session("client", ClientConfig(env=env, base_url=base_url))


def record_document_config(
    *,
    url: str | None = None,
    base_url: str | None = None,
    did: str | None = None,
    wtype: str | None = None,
    wid: str | None = None,
    eid: str | None = None,
    name: str | None = None,
) -> None:
    if url:
        config = DocumentConfig.from_url(url, name=name)
    else:
        merged_base = base_url or BASE_URL
        config = DocumentConfig(
            base_url=merged_base,
            did=did,
            wtype=wtype,
            wid=wid,
            eid=eid,
            name=name,
        )
    _update_session("document", config)


def record_variable_update(element_id: str, expressions: dict[str, str]) -> None:
    session = get_active_session()
    if session is None:
        return

    update = VariableUpdateConfig(element_id=element_id, expressions=expressions)

    if session.preprocessing is None:
        session.preprocessing = PreprocessingConfig(variable_updates=[update])
        return

    for existing in session.preprocessing.variable_updates:
        if existing.element_id == element_id:
            existing.expressions.update(update.expressions)
            break
    else:
        session.preprocessing.variable_updates.append(update)


def record_assembly_config(
    *,
    element_id: str | None,
    configuration: str,
    log_response: bool,
    with_meta_data: bool,
) -> None:
    _update_session(
        "assembly",
        AssemblyConfig(
            element_id=element_id,
            configuration=configuration,
            log_response=log_response,
            with_meta_data=with_meta_data,
        ),
    )


def record_cad_config(max_depth: int) -> None:
    _update_session("cad", CADConfig(max_depth=max_depth))


def record_kinematics_config(use_user_defined_root: bool) -> None:
    _update_session("kinematics", KinematicsConfig(use_user_defined_root=use_user_defined_root))


def record_robot_config(name: str, fetch_mass_properties: bool, robot_type: Literal["urdf", "xml"] = "urdf") -> None:
    _update_session(
        "robot",
        RobotBuildConfig(
            name=name,
            type=robot_type,
            fetch_mass_properties=fetch_mass_properties,
        ),
    )


def record_export_config(
    *,
    file_path: str | None,
    download_assets: bool,
    mesh_dir: str | None,
) -> None:
    _update_session(
        "export",
        ExportConfig(file_path=file_path, download_assets=download_assets, mesh_dir=mesh_dir),
    )


def _ensure_name_entry(
    mapping: dict[str, NameOverrideEntry],
    default_name: str,
    original_name: str | None,
) -> NameOverrideEntry:
    entry = mapping.get(default_name)
    if entry is None:
        entry = NameOverrideEntry(original=original_name, name=default_name)
        mapping[default_name] = entry
        return entry

    if original_name and entry.original is None:
        entry.original = original_name
    if not entry.name:
        entry.name = default_name
    return entry


def record_part_name(default_name: str, original_name: str | None) -> None:
    session = get_active_session()
    _ensure_name_entry(session.names.parts, default_name, original_name)


def record_mate_name(default_name: str, original_name: str | None, limits: dict[str, float] | None = None) -> None:
    session = get_active_session()
    entry = _ensure_name_entry(session.names.mates, default_name, original_name)
    if limits is not None:
        entry.limits = limits


def update_mate_limits(default_name: str, limits: dict[str, float] | None, *, overwrite: bool = False) -> None:
    if limits is None:
        return
    session = get_active_session()
    entry = _ensure_name_entry(session.names.mates, default_name, None)
    if overwrite or entry.limits is None:
        entry.limits = limits


def resolve_part_name(default_name: str) -> str:
    entry = get_active_session().names.parts.get(default_name)
    if entry is None or not entry.name:
        return default_name
    return entry.name


def resolve_mate_name(default_name: str) -> str:
    entry = get_active_session().names.mates.get(default_name)
    if entry is None or not entry.name:
        return default_name
    return entry.name


def resolve_mate_limits(default_name: str) -> dict[str, float] | None:
    """Retrieve joint limits for a mate from the active session config."""
    entry = get_active_session().names.mates.get(default_name)
    if entry is None:
        return None
    return entry.limits


def _auto_save_on_exit() -> None:
    session = _GLOBAL_SESSION

    if session.auto_save_path is None:
        configure_auto_save("ORT.yaml")

    path = session.auto_save_path

    if path is not None:
        session.save(path)
    else:
        logger.warning("No path configured for automatic session saving; skipping.")


atexit.register(_auto_save_on_exit)
