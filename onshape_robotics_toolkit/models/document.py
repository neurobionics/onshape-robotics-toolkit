"""
This module defines data models for Onshape document, workspace, element, and other related entities
retrieved from Onshape REST API responses.

The data models are implemented as Pydantic BaseModel classes, which are used to

    1. Parse JSON responses from the API into Python objects.
    2. Validate the structure and types of the JSON responses.
    3. Provide type hints for better code clarity and autocompletion.

These models ensure that the data received from the API adheres to the expected format and types, facilitating easier
and safer manipulation of the data within the application.

Models:
    - **Document**: Represents an Onshape document, containing the document ID, workspace type, workspace ID,
      and element ID.
    - **DocumentMetaData**: Represents metadata of an Onshape document, containing the default workspace
      information and name.

Supplementary models:
    - **DefaultWorkspace**: Represents the default workspace of an Onshape document, containing the
      workspace ID and type.

Enum:
    - **WorkspaceType**: Enumerates the possible workspace types in Onshape (w, v, m).
    - **MetaWorkspaceType**: Enumerates the possible meta workspace types in Onshape (workspace,
      version, microversion).
"""

from enum import Enum
from typing import Any, Union, cast

import regex as re
from pydantic import BaseModel, Field, field_validator

BASE_URL = "https://cad.onshape.com"

__all__ = ["Document", "WorkspaceType", "parse_url"]


class WorkspaceType(str, Enum):
    """
    Enumerates the possible workspace types in Onshape

    Attributes:
        W (str): Workspace
        V (str): Version
        M (str): Microversion

    Examples:
        >>> WorkspaceType.W
        "w"
        >>> WorkspaceType.M
        "m"
    """

    W = "w"
    V = "v"
    M = "m"


class MetaWorkspaceType(str, Enum):
    """
    Enumerates the possible meta workspace types in Onshape

    Attributes:
        WORKSPACE: workspace
        VERSION: version
        MICROVERSION: microversion

    Properties:
        shorthand: Shorthand representation of the meta workspace type (first letter)

    Examples:
        >>> MetaWorkspaceType.WORKSPACE.shorthand
        "w"
        >>> MetaWorkspaceType.VERSION
        "version"
    """

    WORKSPACE = "workspace"
    VERSION = "version"
    MICROVERSION = "microversion"

    @property
    def shorthand(self) -> str:
        return self.value[0]


# Pattern for matching Onshape document URLs
DOCUMENT_PATTERN = r"(https://[\w\d\.]+)/documents/([\w\d]+)/(w|v|m)/([\w\d]+)/e/([\w\d]+)"


def generate_url(base_url: str, did: str, wtype: str, wid: str, eid: str) -> str:
    """
    Generate Onshape URL from document ID, workspace type, workspace ID, and element ID

    Args:
        did: The unique identifier of the document
        wtype: The type of workspace (w, v, m)
        wid: The unique identifier of the workspace
        eid: The unique identifier of the element

    Returns:
        url: URL to the Onshape document element

    Examples:
        >>> generate_url("a1c1addf75444f54b504f25c", "w", "0d17b8ebb2a4c76be9fff3c7", "a86aaf34d2f4353288df8812")
        "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
    """
    return f"{base_url}/documents/{did}/{wtype}/{wid}/e/{eid}"


def parse_url(url: str) -> tuple[str, str, WorkspaceType, str, str]:
    """
    Parse Onshape URL and return document ID, workspace type, workspace ID, and element ID

    Args:
        url: URL to an Onshape document element

    Returns:
        Tuple containing:
            - base_url: The base URL
            - did: The unique identifier of the document
            - wtype: The type of workspace (w, v, m)
            - wid: The unique identifier of the workspace
            - eid: The unique identifier of the element

    Raises:
        ValueError: If the URL does not match the expected pattern

    Examples:
        >>> parse_url("https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812")
        ("https://cad.onshape.com", "a1c1addf75444f54b504f25c", "w",
         "0d17b8ebb2a4c76be9fff3c7", "a86aaf34d2f4353288df8812")
    """
    pattern = re.match(
        DOCUMENT_PATTERN,
        url,
    )

    if not pattern:
        raise ValueError("Invalid Onshape URL")

    base_url = pattern.group(1)
    did = pattern.group(2)
    wtype = cast(WorkspaceType, pattern.group(3))
    wid = pattern.group(4)
    eid = pattern.group(5)

    return base_url, did, wtype, wid, eid


class Document(BaseModel):
    """
    Represents an Onshape document, containing the document ID, workspace type, workspace ID, and element ID.

    Attributes:
        url: URL to the document element
        did: The unique identifier of the document
        wtype: The type of workspace (w, v, m)
        wid: The unique identifier of the workspace
        eid: The unique identifier of the element
        name: The name of the document

    Methods:
        from_url: Create a Document instance from an Onshape URL

    Examples:
        >>> Document(
        ...     url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812",
        ...     did="a1c1addf75444f54b504f25c",
        ...     wtype="w",
        ...     wid="0d17b8ebb2a4c76be9fff3c7",
        ...     eid="a86aaf34d2f4353288df8812"
        ... )
        Document(
            url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812",
            did="a1c1addf75444f54b504f25c",
            wtype="w",
            wid="0d17b8ebb2a4c76be9fff3c7",
            eid="a86aaf34d2f4353288df8812"
        )
    """

    url: Union[str, None] = Field(None, description="URL to the document element")
    base_url: str = Field(BASE_URL, description="Base URL of the document")
    did: str = Field(..., description="The unique identifier of the document")
    wtype: str = Field(..., description="The type of workspace (w, v, m)")
    wid: str = Field(..., description="The unique identifier of the workspace")
    eid: str = Field(..., description="The unique identifier of the element")
    name: Union[str, None] = Field(None, description="The name of the document")

    def __init__(self, **data: Any):
        super().__init__(**data)
        if self.url is None:
            self.url = generate_url(self.base_url, self.did, self.wtype, self.wid, self.eid)

    @field_validator("did", "wid", "eid")
    def check_ids(cls, value: str) -> str:
        """
        Validate the document, workspace, and element IDs

        Args:
            value: The ID to validate

        Returns:
            value: The validated ID

        Raises:
            ValueError: If the ID is empty or not 24 characters long
        """
        if not value:
            raise ValueError("ID cannot be empty, please check the URL")
        if not len(value) == 24:
            raise ValueError("ID must be 24 characters long, please check the URL")
        return value

    @field_validator("wtype")
    def check_wtype(cls, value: str) -> str:
        """
        Validate the workspace type

        Args:
            value: The workspace type to validate

        Returns:
            value: The validated workspace type

        Raises:
            ValueError: If the workspace type is empty or not one of the valid values
        """
        if not value:
            raise ValueError("Workspace type cannot be empty, please check the URL")

        if value not in WorkspaceType.__members__.values():
            raise ValueError(
                f"Invalid workspace type. Must be one of {WorkspaceType.__members__.values()}, please check the URL"
            )

        return value

    @classmethod
    def from_url(cls, url: str) -> "Document":
        """
        Create a Document instance from an Onshape URL

        Args:
            url: URL to the document element

        Returns:
            Document: The Document instance created from the URL

        Raises:
            ValueError: If the URL does not match the expected pattern

        Examples:
            >>> Document.from_url(
            ...     "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
            ... )
            Document(
                url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812",
                base_url="https://cad.onshape.com",
                did="a1c1addf75444f54b504f25c",
                wtype="w",
                wid="0d17b8ebb2a4c76be9fff3c7",
                eid="a86aaf34d2f4353288df8812"
            )
        """
        base_url, did, wtype, wid, eid = parse_url(url)
        return cls(url=url, base_url=base_url, did=did, wtype=wtype, wid=wid, eid=eid)


class DefaultWorkspace(BaseModel):
    """
    Represents the default workspace of an Onshape document, containing the workspace ID and type.

    JSON:
        ```json
        {
            "id": "739221fb10c88c2bebb456e8"
            "type": "workspace"
        }
        ```

    Attributes:
        id: The unique identifier of the workspace
        type: The type of workspace (workspace, version, microversion)

    Examples:
        >>> DefaultWorkspace(id="739221fb10c88c2bebb456e8", type="workspace")
        DefaultWorkspace(id="739221fb10c88c2bebb456e8", type="workspace")
    """

    id: str = Field(..., description="The unique identifier of the workspace")
    type: MetaWorkspaceType = Field(..., description="The type of workspace (workspace, version, microversion)")


class DocumentMetaData(BaseModel):
    """
    Represents metadata of an Onshape document, containing the default workspace information and name.

    JSON:
        ```json
        {
            "defaultWorkspace": {
                "id": "739221fb10c88c2bebb456e8",
                "type": "workspace"
            },
            "name": "Document Name",
            "id": "a1c1addf75444f54b504f25c"
        }
        ```

    Attributes:
        defaultWorkspace: Default workspace information
        name: The name of the document
        id: The unique identifier of the document

    Examples:
        >>> DocumentMetaData(
        ...     defaultWorkspace=DefaultWorkspace(id="739221fb10c88c2bebb456e8", type="workspace"),
        ...     name="Document Name",
        ...     id="a1c1addf75444f54b504f25c"
        ... )
        DocumentMetaData(
            defaultWorkspace=DefaultWorkspace(id="739221fb10c88c2bebb456e8", type="workspace"),
            name="Document Name",
            id="a1c1addf75444f54b504f25c"
        )
    """

    defaultWorkspace: DefaultWorkspace = Field(..., description="Default workspace information")
    name: str = Field(..., description="The name of the document")
    id: str = Field(..., description="The unique identifier of the document")
