"""
This module provides a client class and other utilities to interact with the Onshape API.

Class:
    - **Client**: Provides access to the Onshape REST API.
    - **Part**: Represents a part within an assembly, including its properties and configuration.
    - **PartInstance**: Represents an instance of a part within an assembly.

Enum:
    - **HTTP**: Enumerates the possible HTTP methods (GET, POST, DELETE).

"""

import asyncio
import atexit
import base64
import datetime
import hashlib
import hmac
import io
import os
import secrets
import string
import time
from enum import Enum
from typing import Any, BinaryIO, Optional, Union
from urllib.parse import parse_qs, urlencode, urlparse

import lxml.etree as ET
import numpy as np
import requests
import stl
from loguru import logger

from onshape_robotics_toolkit.config import (
    record_assembly_config,
    record_client_config,
    record_document_config,
    record_variable_update,
)
from onshape_robotics_toolkit.mesh import transform_mesh
from onshape_robotics_toolkit.models.assembly import Assembly, Features, RootAssembly
from onshape_robotics_toolkit.models.document import BASE_URL, Document, DocumentMetaData, generate_url
from onshape_robotics_toolkit.models.element import Element
from onshape_robotics_toolkit.models.mass import MassProperties
from onshape_robotics_toolkit.models.variable import Variable
from onshape_robotics_toolkit.utilities.helpers import (
    get_sanitized_name,
    load_key_from_dotenv,
    load_key_from_environment,
)

CURRENT_DIR = os.getcwd()
MESHES_DIR = "meshes"
ONSHAPE_KEY_NAMES = ["ONSHAPE_ACCESS_KEY", "ONSHAPE_SECRET_KEY"]

__all__ = ["HTTP", "Client"]

# TODO: Add asyncio support for async requests


class HTTP(str, Enum):
    """
    Enumerates the possible HTTP methods.

    Attributes:
        GET (str): HTTP GET method
        POST (str): HTTP POST method
        DELETE (str): HTTP DELETE method

    Examples:
        >>> HTTP.GET
        'get'
        >>> HTTP.POST
        'post'
    """

    GET = "get"
    POST = "post"
    DELETE = "delete"


def load_env_variables(env: Union[str, None]) -> tuple[str, str]:
    """Load access and secret keys required for Onshape API requests.

    Args:
        env: Path to the environment file containing the access and secret keys. If
            `None`, the environment variables are loaded from the system environment
            with `os.getenv`. If `not None`, the environment variables are loaded from
            the file with `dotenv.get_key`.

    Returns:
        tuple[str, str]: Access and secret keys

    Raises:
        FileNotFoundError: If the environment file is not found
        ValueError: If the required environment variables are missing

    Examples:
        >>> load_env_variables(".env")
        ('asdagflkdfjsdlfkdfjlsdf', 'asdkkjdnknsdgkjsdguoiuosdg')
    """

    if env is not None:
        return (load_key_from_dotenv(env, ONSHAPE_KEY_NAMES[0]), load_key_from_dotenv(env, ONSHAPE_KEY_NAMES[1]))
    else:
        return (load_key_from_environment(ONSHAPE_KEY_NAMES[0]), load_key_from_environment(ONSHAPE_KEY_NAMES[1]))


def make_nonce() -> str:
    """
    Generate a unique ID for the request, 25 chars in length

    Returns:
        Cryptographic nonce string for the API request

    Examples:
        >>> make_nonce()
        'a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p'
    """

    chars = string.digits + string.ascii_letters
    nonce = "".join(secrets.choice(chars) for i in range(25))
    logger.debug(f"nonce created: {nonce}")

    return nonce


class Client:
    """
    Represents a client for the Onshape REST API with methods to interact with the API.

    Args:
        env (str, default='./.env'): Path to the environment file containing the access and secret keys
        base_url (str, default='https://cad.onshape.com'): Base URL for the Onshape API

    Attributes:
        api_call_count (int): Number of successful API calls made (read-only property)

    Methods:
        get_document_metadata: Get details for a specified document.
        get_elements: Get list of elements in a document.
        get_variables: Get list of variables in a variable studio.
        set_variables: Set variables in a variable studio.
        get_assembly: Get assembly data for a specified document / workspace / assembly.
        download_part_stl: Download an STL file from a part studio.
        get_mass_property: Get mass properties for a part in a part studio.
        request: Issue a request to the Onshape API.
        reset_api_call_count: Reset the API call counter to zero.

    Note:
        The client automatically tracks API calls and logs a summary when the program
        exits (using atexit). This works reliably even when scripts crash or encounter
        network errors.

    Examples:
        >>> client = Client(
        ...     env=".env",
        ... )
        >>> document_meta_data = client.get_document_metadata("document_id")
        >>> print(f"API calls made: {client.api_call_count}")
        API calls made: 1
    """

    def __init__(self, env: Union[str, None] = None, base_url: str = BASE_URL):
        """
        Initialize the Onshape API client.

        Args:
            env: Path to the environment file containing the access and secret keys
            base_url: Base URL for the Onshape API

        Examples:
            >>> client = Client(
            ...     env=".env",
            ... )
        """

        self._url = base_url
        self._access_key, self._secret_key = load_env_variables(env)
        self._api_call_count = 0
        logger.info(f"Onshape API initialized with env file: {env}")
        record_client_config(env=env, base_url=base_url)

        # Register cleanup function to log API usage on exit
        atexit.register(self._log_final_api_count)

    def _log_final_api_count(self) -> None:
        """
        Log the final API call count when the program exits.

        This method is registered with atexit and will be called on normal program
        termination. It provides visibility into API usage even when scripts error
        out or network issues cause early termination.
        """
        if self._api_call_count > 0:
            logger.info(f"Client session ended → Total API calls made: {self._api_call_count}")

    def set_base_url(self, base_url: str) -> None:
        """
        Set the base URL for the Onshape API.

        Args:
            base_url: Base URL for the Onshape API

        Examples:
            >>> client.set_base_url("https://cad.onshape.com")
        """
        self._url = base_url

    @property
    def api_call_count(self) -> int:
        """
        Get the current API call count.

        Returns:
            The number of successful API calls made by this client

        Examples:
            >>> client.api_call_count
            42
        """
        return self._api_call_count

    def reset_api_call_count(self) -> None:
        """
        Reset the API call counter to zero.

        Examples:
            >>> client.reset_api_call_count()
            >>> client.api_call_count
            0
        """
        self._api_call_count = 0
        logger.info("API call counter reset to 0")

    def get_document_metadata(self, did: str) -> DocumentMetaData:
        """
        Get meta data for a specified document.

        Args:
            did: The unique identifier of the document.

        Returns:
            Meta data for the specified document as a DocumentMetaData object or None if the document is not found

        Examples:
            >>> document_meta_data = client.get_document_metadata("document_id
            >>> print(document_meta_data)
            DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="739221fb10c88c2bebb456e8", type="workspace"),
                name="Document Name",
                id="a1c1addf75444f54b504f25c"
            )
        """
        if len(did) != 24:
            raise ValueError(f"Invalid document ID: {did}")

        res = self.request(HTTP.GET, "/api/documents/" + did)

        if res.status_code == 404:
            raise ValueError(f"Document does not exist: {did}")
        elif res.status_code == 403:
            raise ValueError(f"Access forbidden for document: {did}")

        document = DocumentMetaData.model_validate(res.json())
        document.name = get_sanitized_name(document.name)

        return document

    def get_elements(self, did: str, wtype: str, wid: str) -> dict[str, Element]:
        """
        Get a list of all elements in a document.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.

        Returns:
            A dictionary of element name and Element object pairs.

        Examples:
            >>> elements = client.get_elements(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7"
            ... )
            >>> print(elements)
            {
                "wheelAndFork": Element(id='0b0c209535554345432581fe', name='wheelAndFork', elementType='PARTSTUDIO',
                                         microversionId='9b3be6165c7a2b1f6dd61305'),
                "frame": Element(id='0b0c209535554345432581fe', name='frame', elementType='PARTSTUDIO',
                                 microversionId='9b3be6165c7a2b1f6dd61305')
            }
        """

        # /documents/d/{did}/{wvm}/{wvmid}/elements
        request_path = "/api/documents/d/" + did + "/" + wtype + "/" + wid + "/elements"
        response = self.request(
            HTTP.GET,
            request_path,
        )

        if response.status_code == 404:
            logger.error(f"Elements not found for document: {did}")
            return {}

        elif response.status_code == 403:
            logger.error(f"Access forbidden for document: {did}")
            return {}

        return {element["name"]: Element.model_validate(element) for element in response.json()}

    def get_variables(self, did: str, wid: str, eid: str) -> dict[str, Variable]:
        """
        Get a list of variables in a variable studio within a document.

        Args:
            did: The unique identifier of the document.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the variable studio.

        Returns:
            A dictionary of variable name and Variable object pairs.

        Examples:
            >>> variables = client.get_variables(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="cba5e3ca026547f34f8d9f0f"
            ... )
            >>> print(variables)
            {
                "forkAngle": Variable(
                    type='ANGLE',
                    name='forkAngle',
                    value=None,
                    description='Fork angle for front wheel assembly in deg',
                    expression='15 deg'
                )
            }
        """
        request_path = "/api/variables/d/" + did + "/w/" + wid + "/e/" + eid + "/variables"

        _variables_json = self.request(
            HTTP.GET,
            request_path,
        ).json()

        return {variable["name"]: Variable.model_validate(variable) for variable in _variables_json[0]["variables"]}

    def set_variables(self, did: str, wid: str, eid: str, variables: dict[str, Variable]) -> requests.Response:
        """
        Set values for variables of a variable studio in a document.

        Args:
            did: The unique identifier of the document.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the variable studio.
            variables: A dictionary of variable name and Variable pairs

        Returns:
            requests.Response: Response from Onshape API after setting the variables.

        Examples:
            >>> variables = {
            ...     "forkAngle": "15 deg",
            ...     "wheelRadius": "0.5 m"
            ... }
            >>> client.set_variables(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="cba5e3ca026547f34f8d9f0f",
            ...     variables=variables
            ... )
            <Response [200]>
        """
        for var_name, variable in variables.items():
            if not isinstance(variable, Variable):
                raise TypeError(f"Variable '{var_name}' is not a valid Variable object.")

        payload = [variable.model_dump() for variable in variables.values()]

        # api/v9/variables/d/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/cba5e3ca026547f34f8d9f0f/variables
        request_path = "/api/variables/d/" + did + "/w/" + wid + "/e/" + eid + "/variables"

        response = self.request(
            HTTP.POST,
            request_path,
            body=payload,
        )

        record_variable_update(element_id=eid, expressions=variables)

        return response

    def get_features(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
    ) -> Features:
        """
        Get all assembly features

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the assembly.
        """
        request_path = f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/features"
        response_json = self.request(HTTP.GET, request_path, log_response=True).json()
        return Features.model_validate(response_json)

    def get_assembly_name(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        configuration: str = "default",
    ) -> Optional[str]:
        """
        Get assembly name for a specified document / workspace / assembly.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the assembly.
            configuration: The configuration of the assembly.

        Returns:
            str: Assembly name

        Examples:
            >>> assembly_name = client.get_assembly_name(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812"
            ... )
            >>> print(assembly_name)
            "Assembly Name"
        """
        request_path = "/api/metadata/d/" + did + "/" + wtype + "/" + wid + "/e/" + eid
        result_json = self.request(
            HTTP.GET,
            request_path,
            query={
                "inferMetadataOwner": "false",
                "includeComputedProperties": "false",
                "includeComputedAssemblyProperties": "false",
                "thumbnail": "false",
                "configuration": configuration,
            },
            log_response=False,
        ).json()

        name = None
        try:
            name = result_json["properties"][0]["value"]
            name = get_sanitized_name(name)

        except KeyError:
            logger.warning(f"Assembly name not found for document: {did}")

        return name

    def get_root_assembly(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        configuration: str = "default",
        with_mass_properties: bool = False,
        log_response: bool = True,
        with_meta_data: bool = True,
    ) -> RootAssembly:
        """
        Get root assembly data for a specified document / workspace / element.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the element.
            configuration: The configuration of the assembly.
            with_mass_properties: Whether to include mass properties in the assembly data.
            log_response: Log the response from the API request.
            with_meta_data: Whether to include meta data in the assembly data.

        Returns:
            RootAssembly: RootAssembly object containing the root assembly data

        Examples:
            >>> root_assembly = client.get_root_assembly(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812",
            ...     configuration="default",
            ...     with_mass_properties=True,
            ...     log_response=False,
            ...     with_meta_data=True
            ... )
            >>> print(root_assembly)
            RootAssembly(
                instances=[...],
                patterns=[...],
                features=[...],
                occurrences=[...],
                fullConfiguration="default",
                configuration="default",
                documentId="a1c1addf75444f54b504f25c",
                elementId="0b0c209535554345432581fe",
                documentMicroversion="349f6413cafefe8fb4ab3b07",
            )
        """
        request_path = "/api/assemblies/d/" + did + "/" + wtype + "/" + wid + "/e/" + eid
        res = self.request(
            HTTP.GET,
            request_path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "false",
                "configuration": configuration,
            },
            log_response=log_response,
        )

        if res.status_code == 401:
            logger.warning(f"Unauthorized access to document: {did}")
            logger.warning("Please check the API keys in your env file.")
            exit(1)

        if res.status_code == 404:
            logger.error(f"Assembly not found: {did}")
            logger.error(
                generate_url(
                    base_url=self._url,
                    did=did,
                    wtype=wtype,
                    wid=wid,
                    eid=eid,
                )
            )
            exit(1)

        assembly_json = res.json()
        assembly = RootAssembly.model_validate(assembly_json["rootAssembly"])

        if with_mass_properties:
            assembly.MassProperty = self.get_assembly_mass_properties(
                did=did,
                wid=wid,
                eid=eid,
                wtype=wtype,
            )

        if with_meta_data:
            assembly.documentMetaData = self.get_document_metadata(did)

        return assembly

    def get_assembly(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        configuration: str = "default",
        log_response: bool = True,
        with_meta_data: bool = True,
    ) -> Assembly:
        """
        Get assembly data for a specified document / workspace / assembly.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the assembly.
            configuration: The configuration of the assembly.
            log_response: Log the response from the API request.
            with_meta_data: Include meta data in the assembly data.

        Returns:
            Assembly: Assembly object containing the assembly data

        Examples:
            >>> assembly = client.get_assembly(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812"
            ... )
            >>> print(assembly)
            Assembly(
                rootAssembly=RootAssembly(
                    instances=[...],
                    patterns=[...],
                    features=[...],
                    occurrences=[...],
                    fullConfiguration="default",
                    configuration="default",
                    documentId="a1c1addf75444f54b504f25c",
                    elementId="0b0c209535554345432581fe",
                    documentMicroversion="349f6413cafefe8fb4ab3b07",
                ),
                subAssemblies=[...],
                parts=[...],
                partStudioFeatures=[...],
                document=Document(
                    url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812",
                    did="a1c1addf75444f54b504f25c",
                    wtype="w",
                    wid="0d17b8ebb2a4c76be9fff3c7",
                    eid="a86aaf34d2f4353288df8812"
                )
            )
        """
        request_path = "/api/assemblies/d/" + did + "/" + wtype + "/" + wid + "/e/" + eid
        res = self.request(
            HTTP.GET,
            request_path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
            log_response=log_response,
        )

        if res.status_code == 401 or res.status_code == 403:
            logger.warning(f"Unauthorized access to document: {did}")
            logger.warning("Please check the API keys in your env file.")
            exit(1)

        if res.status_code == 404:
            logger.error(f"Assembly not found: {did}")
            logger.error(
                generate_url(
                    base_url=self._url,
                    did=did,
                    wtype=wtype,
                    wid=wid,
                    eid=eid,
                )
            )
            exit(1)

        # Clean numerical values from API response before validation
        from onshape_robotics_toolkit.utilities.helpers import clean_json_numerics

        assembly_data = clean_json_numerics(res.json(), threshold=1e-10, decimals=8)
        assembly = Assembly.model_validate(assembly_data)
        document = Document(did=did, wtype=wtype, wid=wid, eid=eid)
        assembly.document = document

        if with_meta_data:
            assembly.name = self.get_assembly_name(did, wtype, wid, eid, configuration)
            document_meta_data = self.get_document_metadata(did)
            assembly.document.name = document_meta_data.name

        record_document_config(
            url=getattr(assembly.document, "url", None),
            base_url=self._url,
            did=did,
            wtype=wtype,
            wid=wid,
            eid=eid,
            name=getattr(assembly.document, "name", None),
        )
        record_assembly_config(
            element_id=eid,
            configuration=configuration,
            log_response=log_response,
            with_meta_data=with_meta_data,
        )

        return assembly

    def download_assembly_stl(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        buffer: BinaryIO,
        configuration: str = "default",
    ) -> Optional[BinaryIO]:
        """
        Download an STL file from an assembly. The file is written to the buffer.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the element.
            buffer: BinaryIO object to write the STL file to.
            configuration: The configuration of the assembly.

        """
        req_headers = {"Accept": "application/vnd.onshape.v1+octet-stream"}
        request_path = f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/translations"

        # Initiate the translation
        payload = {
            "formatName": "STL",
            "storeInDocument": "false",
        }
        response = self.request(
            HTTP.POST,
            path=request_path,
            body=payload,
            log_response=False,
        )

        if response.status_code == 200:
            job_info = response.json()
            translation_id = job_info.get("id")
            if not translation_id:
                logger.error("Translation job ID not found in response.")
                return None

            status_path = f"/api/translations/{translation_id}"
            while True:
                status_response = self.request(HTTP.GET, path=status_path)
                if status_response.status_code != 200:
                    logger.error(f"Failed to get translation status: {status_response.text}")
                    return None

                status_info = status_response.json()
                request_state = status_info.get("requestState")
                logger.info(f"Current status: {request_state}")
                if request_state == "DONE":
                    logger.info("Translation job completed.")
                    break
                elif request_state == "FAILED":
                    logger.error("Translation job failed.")
                    return None
                time.sleep(1)

            fid = status_info.get("resultExternalDataIds")[0]
            data_path = f"/api/documents/d/{did}/externaldata/{fid}"

            download_response = self.request(
                HTTP.GET,
                path=data_path,
                headers=req_headers,
                log_response=False,
            )
            if download_response.status_code == 200:
                buffer.write(download_response.content)
                logger.info("STL file downloaded successfully.")
                return buffer
            else:
                logger.error(f"Failed to download STL file: {download_response.text}")
                return None

        else:
            logger.info(f"Failed to download assembly: {response.status_code} - {response.text}")
            logger.info(
                generate_url(
                    base_url=self._url,
                    did=did,
                    wtype=wtype,
                    wid=wid,
                    eid=eid,
                )
            )

        return buffer

    def download_part_stl(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        partID: str,
        buffer: BinaryIO,
    ) -> BinaryIO:
        """
        Download an STL file from a part studio. The file is written to the buffer.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the element.
            partID: The unique identifier of the part.
            buffer: BinaryIO object to write the STL file to.

        Returns:
            BinaryIO: BinaryIO object containing the STL file

        Examples:
            >>> with io.BytesIO() as buffer:
            ...     client.download_part_stl(
            ...         "a1c1addf75444f54b504f25c",
            ...         "0d17b8ebb2a4c76be9fff3c7",
            ...         "a86aaf34d2f4353288df8812",
            ...         "0b0c209535554345432581fe",
            ...         buffer,
            ...         "w",
            ...         "0d17b8ebb2a4c76be9fff3c7"
            ...     )
            >>> buffer.seek(0)
            >>> raw_mesh = stl.mesh.Mesh.from_file(None, fh=buffer)
            >>> raw_mesh.save("mesh.stl")
        """
        # TODO: version id seems to always work, should this default behavior be changed?
        req_headers = {"Accept": "application/vnd.onshape.v1+octet-stream"}
        request_path = f"/api/parts/d/{did}/{wtype}/{wid}/e/{eid}/partid/{partID}/stl"
        _query = {
            "mode": "binary",
            "grouping": True,
            "units": "meter",
        }
        response = self.request(
            HTTP.GET,
            path=request_path,
            headers=req_headers,
            query=_query,
            log_response=False,
        )
        if response.status_code == 200:
            buffer.write(response.content)
        else:
            url = generate_url(
                base_url=self._url,
                did=did,
                wtype=wtype,
                wid=wid,
                eid=eid,
            )
            logger.info(f"{url}")
            logger.info(f"Failed to download STL file: {response.status_code} - {response.text}")

        return buffer

    def get_assembly_mass_properties(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
    ) -> MassProperties:
        """
        Get mass properties of a rigid assembly in a document.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the rigid assembly.

        Returns:
            MassProperties object containing the mass properties of the assembly.

        Examples:
            >>> mass_properties = client.get_assembly_mass_properties(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812",
            ...     wtype="w"
            ... )
            >>> print(mass_properties)
            MassProperties(
                volume=[0.003411385108378978, 0.003410724395374695, 0.0034120458213832646],
                mass=[9.585992154544929, 9.584199206938452, 9.587785102151415],
                centroid=[...],
                inertia=[...],
                principalInertia=[0.09944605933465941, 0.09944605954654827, 0.19238058837442526],
                principalAxes=[...]
            )
        """
        request_path = f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/massproperties"
        res = self.request(HTTP.GET, request_path, log_response=False)

        if res.status_code == 404:
            url = generate_url(
                base_url=self._url,
                did=did,
                wtype=wtype,
                wid=wid,
                eid=eid,
            )
            raise ValueError(f"Assembly: {url} does not have a mass property")

        return MassProperties.model_validate(res.json())

    def get_mass_property(
        self,
        did: str,
        wtype: str,
        wid: str,
        eid: str,
        partID: str,
    ) -> MassProperties:
        """
        Get mass properties of a part in a part studio.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the element.
            partID: The identifier of the part.

        Returns:
            MassProperties object containing the mass properties of the part.

        Examples:
            >>> mass_properties = client.get_mass_property(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812",
            ...     partID="0b0c209535554345432581fe"
            ...     wtype="w"
            ... )
            >>> print(mass_properties)
            MassProperties(
                volume=[0.003411385108378978, 0.003410724395374695, 0.0034120458213832646],
                mass=[9.585992154544929, 9.584199206938452, 9.587785102151415],
                centroid=[...],
                inertia=[...],
                principalInertia=[0.09944605933465941, 0.09944605954654827, 0.19238058837442526],
                principalAxes=[...]
            )
        """
        # TODO: version id seems to always work, should this default behavior be changed?
        request_path = f"/api/parts/d/{did}/{wtype}/{wid}/e/{eid}/partid/{partID}/massproperties"
        res = self.request(HTTP.GET, request_path, {"useMassPropertiesOverrides": True}, log_response=False)

        if res.status_code == 404:
            # TODO: There doesn't seem to be a way to assign material to a part currently
            # It is possible that the workspace got deleted
            url = generate_url(
                base_url=self._url,
                did=did,
                wtype=wtype,
                wid=wid,
                eid=eid,
            )
            raise ValueError(f"Part: {url} does not have a material assigned or the part is not found")

        elif res.status_code == 429:
            raise ValueError(f"Too many requests, please retry after {res.headers['Retry-After']} seconds")

        resonse_json = res.json()

        if "bodies" not in resonse_json:
            raise KeyError(f"Bodies not found in response, broken part? {partID}")

        return MassProperties.model_validate(resonse_json["bodies"][partID])

    def request(
        self,
        method: HTTP,
        path: str,
        query: Optional[dict[str, Any]] = None,
        headers: Optional[dict[str, Any]] = None,
        body: Optional[Union[dict[str, Any], list[dict[str, Any]]]] = None,
        base_url: Optional[str] = None,
        log_response: bool = True,
        timeout: int = 50,
    ) -> requests.Response:
        """
        Send a request to the Onshape API.

        Args:
            method: HTTP method (GET, POST, DELETE)
            path: URL path for the request
            query: Query string in key-value pairs
            headers: Additional headers for the request
            body: Body of the request
            base_url: Base URL for the request
            log_response: Log the response from the API request
            timeout: Timeout for the request in seconds
        Returns:
            requests.Response: Response from the Onshape API request
        """
        if query is None:
            query = {}
        if headers is None:
            headers = {}
        if base_url is None:
            base_url = self._url

        req_headers = self._make_headers(method, path, query, headers)
        url = self._build_url(base_url, path, query)

        logger.debug(f"Request body: {body}")
        logger.debug(f"Request headers: {req_headers}")
        logger.debug(f"Request URL: {url}")

        res = self._send_request(method, url, req_headers, body, timeout)

        if res.status_code == 307:
            return self._handle_redirect(res, method, headers, log_response)
        else:
            if log_response:
                self._log_response(res)

        # Increment API call counter for successful responses (2xx status codes)
        if 200 <= res.status_code < 300:
            self._api_call_count += 1

        return res

    def _build_url(self, base_url: str, path: str, query: dict[str, Any]) -> str:
        """
        Build the URL for the request.

        Args:
            base_url: The base URL for the request.
            path: The path for the request.
            query: The query string for the request.

        Returns:
            The URL for the request.
        """
        return base_url + path + "?" + urlencode(query)

    def _send_request(
        self,
        method: HTTP,
        url: str,
        headers: dict[str, Any],
        body: Optional[Union[dict[str, Any], list[dict[str, Any]]]],
        timeout: int,
    ) -> requests.Response:
        """
        Send the request to the Onshape API.

        Args:
            method: The HTTP method for the request.
            url: The URL for the request.
            headers: The headers for the request.
            body: The body for the request.
            timeout: The timeout for the request in seconds.

        Returns:
            The response from the Onshape API request.
        """
        return requests.request(
            method,
            url,
            headers=headers,
            json=body,
            allow_redirects=False,
            stream=True,
            timeout=timeout,  # Specify an appropriate timeout value in seconds
        )

    def _handle_redirect(
        self,
        res: requests.Response,
        method: HTTP,
        headers: dict[str, Any],
        log_response: bool = True,
    ) -> requests.Response:
        """
        Handle a redirect response from the Onshape API.

        Args:
            res: The response from the Onshape API request.
            method: The HTTP method for the request.
            headers: The headers for the request.
            log_response: Whether to log the response from the API request.

        Returns:
            The response from the Onshape API request.
        """
        location = urlparse(res.headers["Location"])
        querystring = parse_qs(location.query)

        logger.debug(f"Request redirected to: {location.geturl()}")

        new_query = {key: querystring[key][0] for key in querystring}
        new_base_url = location.scheme + "://" + location.netloc

        return self.request(
            method, location.path, query=new_query, headers=headers, base_url=new_base_url, log_response=log_response
        )

    def _log_response(self, res: requests.Response) -> None:
        """
        Log the response from the Onshape API request.

        Args:
            res: The response from the Onshape API request.
        """
        try:
            if not 200 <= res.status_code <= 206:
                logger.debug(f"Request failed, details: {res.text}")
            else:
                logger.debug(f"Request succeeded, details: {res.text}")
        except UnicodeEncodeError as e:
            logger.error(f"UnicodeEncodeError: {e}")

    def _make_auth(
        self,
        method: HTTP,
        date: str,
        nonce: str,
        path: str,
        query: Optional[dict[str, Any]] = None,
        ctype: str = "application/json",
    ) -> str:
        """
        Make the authentication header for the Onshape API request.

        Args:
            method: The HTTP method for the request.
            date: The date for the request.
            nonce: The nonce for the request.
            path: The path for the request.
            query: The query string for the request.
            ctype: The content type for the request.

        Returns:
            The authentication header for the Onshape API request.
        """
        if query is None:
            query = {}
        query_string = urlencode(query)

        hmac_str = (
            str(method + "\n" + nonce + "\n" + date + "\n" + ctype + "\n" + path + "\n" + query_string + "\n")
            .lower()
            .encode("utf-8")
        )

        signature = base64.b64encode(
            hmac.new(self._secret_key.encode("utf-8"), hmac_str, digestmod=hashlib.sha256).digest()
        )
        auth = "On " + self._access_key + ":HmacSHA256:" + signature.decode("utf-8")

        logger.debug(f"query: {query_string}, hmac_str: {hmac_str!r}, signature: {signature!r}, auth: {auth}")

        return auth

    def _make_headers(
        self,
        method: HTTP,
        path: str,
        query: Optional[dict[str, Any]] = None,
        headers: Optional[dict[str, Any]] = None,
    ) -> dict[str, Any]:
        """
        Make the headers for the Onshape API request.

        Args:
            method: The HTTP method for the request.
            path: The path for the request.
            query: The query string for the request.
            headers: The headers for the request.

        Returns:
            The headers for the Onshape API request.
        """
        if headers is None:
            headers = {}
        if query is None:
            query = {}
        date = datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S GMT")
        nonce = make_nonce()
        ctype = headers.get("Content-Type") if headers.get("Content-Type") else "application/json"

        auth = self._make_auth(method, date, nonce, path, query=query, ctype=ctype or "application/json")

        req_headers = {
            "Content-Type": "application/json",
            "Date": date,
            "On-Nonce": nonce,
            "Authorization": auth,
            "User-Agent": "Onshape Python Sample App",
            "Accept": "application/json",
        }

        # add in user-defined headers
        for h in headers:
            req_headers[h] = headers[h]

        return req_headers

    @property
    def base_url(self) -> str:
        """
        Get the base URL for the Onshape API request.

        Returns:
            The base URL for the Onshape API request.
        """
        return self._url


class Asset:
    """
    Represents a set of parameters required to download a link from Onshape.
    """

    def __init__(
        self,
        file_name: str,
        did: str = "",
        wtype: str = "",
        wid: str = "",
        eid: str = "",
        client: Optional[Client] = None,
        transform: Optional[np.ndarray] = None,
        is_rigid_assembly: bool = False,
        partID: Optional[str] = None,
        is_from_file: bool = False,
        mesh_dir: Optional[str] = None,
    ) -> None:
        """
        Initialize the Asset object.

        Args:
            did: The unique identifier of the document.
            wtype: The type of workspace.
            wid: The unique identifier of the workspace.
            eid: The unique identifier of the element.
            client: Onshape API client object.
            transform: Transformation matrix to apply to the mesh.
            file_name: Name of the mesh file.
            is_rigid_assembly: Whether the element is a rigid assembly.
            partID: The unique identifier of the part.
            is_from_file: Whether the asset is from a file.
            mesh_dir: Optional custom directory for mesh files. If None, uses CURRENT_DIR/MESHES_DIR.
        """
        self.did = did
        self.wtype = wtype
        self.wid = wid
        self.eid = eid
        self.client = client
        self.transform = transform
        self.file_name = file_name
        self.is_rigid_assembly = is_rigid_assembly
        self.partID = partID
        self.is_from_file = is_from_file
        self.mesh_dir = mesh_dir
        self.robot_file_dir: Optional[str] = None  # Directory where robot file is saved, for relative path calculation

        self._file_path: Optional[str] = None

    @property
    def absolute_path(self) -> str:
        """
        Returns the file path of the mesh file.

        Returns:
            The file path of the mesh file.
        """
        if self.is_from_file:
            if self._file_path is None:
                raise ValueError("File path is not set for file-based asset")
            return self._file_path

        # Determine the mesh directory to use (custom or default for backwards compatibility)
        mesh_directory = self.mesh_dir if self.mesh_dir is not None else os.path.join(CURRENT_DIR, MESHES_DIR)

        # Create the directory if it doesn't exist
        if not os.path.exists(mesh_directory):
            os.makedirs(mesh_directory)

        return os.path.join(mesh_directory, self.file_name)

    @property
    def relative_path(self) -> str:
        """
        Returns the relative path of the mesh file.

        Returns:
            The relative path of the mesh file with forward slashes (cross-platform compatible).
        """
        # Calculate relative path from robot file directory if specified, otherwise from CURRENT_DIR
        base_dir = self.robot_file_dir if self.robot_file_dir is not None else CURRENT_DIR

        # Use forward slashes for cross-platform compatibility in URDF/MJCF files
        rel_path = os.path.relpath(self.absolute_path, base_dir)
        return rel_path.replace(os.sep, "/")

    async def download(self) -> None:
        """
        Asynchronously download the mesh file from Onshape, transform it, and save it to a file.

        Examples:
            >>> asset = Asset(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812",
            ...     client=client,
            ...     transform=np.eye(4),
            ...     file_name="mesh.stl",
            ...     is_rigid_assembly=True
            ... )
            >>> await asset.download()
        """
        logger.info(f"Downloading {self.file_name}")
        if self.client is None:
            raise ValueError("Client is required for downloading meshes")
        try:
            with io.BytesIO() as buffer:
                if not self.is_rigid_assembly:
                    if self.partID is None:
                        raise ValueError("Part ID is required for downloading part STL")  # noqa: TRY301
                    await asyncio.to_thread(
                        self.client.download_part_stl,
                        did=self.did,
                        wtype=self.wtype,
                        wid=self.wid,
                        eid=self.eid,
                        partID=self.partID,
                        buffer=buffer,
                    )
                else:
                    await asyncio.to_thread(
                        self.client.download_assembly_stl,
                        did=self.did,
                        wtype=self.wtype,
                        wid=self.wid,
                        eid=self.eid,
                        buffer=buffer,
                    )

                buffer.seek(0)

                raw_mesh = stl.mesh.Mesh.from_file(None, fh=buffer)
                transformed_mesh = transform_mesh(raw_mesh, self.transform) if self.transform is not None else raw_mesh
                transformed_mesh.save(self.absolute_path)

                logger.debug(f"Mesh file saved: {self.absolute_path}")
        except Exception as e:
            logger.error(f"Failed to download {self.file_name}: {e}")

    def to_mjcf(self, root: Any) -> None:
        """
        Returns the XML representation of the asset, which is a mesh file.

        Args:
            root: The root element of the XML tree.

        Examples:
            >>> asset = Asset(
            ...     did="a1c1addf75444f54b504f25c",
            ...     wtype="w",
            ...     wid="0d17b8ebb2a4c76be9fff3c7",
            ...     eid="a86aaf34d2f4353288df8812",
            ...     client=client,
            ...     transform=np.eye(4),
            ...     file_name="mesh.stl",
            ...     is_rigid_assembly=True
            ... )
            >>> asset.to_mjcf()
            <mesh name="Part-1-1" file="Part-1-1.stl" />
        """
        asset = ET.Element("mesh") if root is None else ET.SubElement(root, "mesh")
        asset.set("name", self.file_name.split(".")[0])
        asset.set("file", self.relative_path)

    @classmethod
    def from_file(cls, file_path: str) -> "Asset":
        """
        Create an Asset object from a mesh file.

        Args:
            file_path: Path to the mesh file.

        Returns:
            Asset: Asset object representing the mesh file.

        Examples:
            >>> asset = Asset.from_file("mesh.stl", client)
        """
        file_name = os.path.basename(file_path)
        asset = cls(
            file_name=file_name.split(".")[0],
            is_from_file=True,
        )

        asset._file_path = file_path
        return asset
