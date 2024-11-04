import base64
import datetime
import hashlib
import hmac
import os
import secrets
import string
from enum import Enum
from typing import Any, BinaryIO, Optional, Union
from urllib.parse import parse_qs, urlencode, urlparse

import requests
from dotenv import load_dotenv

from onshape_api.log import LOG_LEVEL, LOGGER
from onshape_api.models.assembly import Assembly
from onshape_api.models.document import DocumentMetaData
from onshape_api.models.element import Element
from onshape_api.models.mass import MassModel
from onshape_api.models.variable import Variable

__all__ = ["Client", "BASE_URL", "HTTP"]

# TODO: Add asyncio support for async requests

BASE_URL = "https://cad.onshape.com"


class HTTP(str, Enum):
    GET = "get"
    POST = "post"
    DELETE = "delete"


def load_env_variables(env: str) -> tuple[str, str]:
    """
    Load environment variables from the specified .env file.

    Args:
        env (str): Path to the .env file.

    Returns:
        tuple: A tuple containing the URL, ACCESS_KEY, and SECRET_KEY.

    Raises:
        FileNotFoundError: If the .env file does not exist.
        ValueError: If any of the required environment variables are missing.
    """
    if not os.path.isfile(env):
        raise FileNotFoundError(f"{env} file not found")

    load_dotenv(env)

    access_key = os.getenv("ACCESS_KEY")
    secret_key = os.getenv("SECRET_KEY")

    if not access_key or not secret_key:
        missing_vars = [var for var in ["ACCESS_KEY", "SECRET_KEY"] if not os.getenv(var)]
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    return access_key, secret_key


def make_nonce() -> str:
    """
    Generate a unique ID for the request, 25 chars in length

    Returns:
        - str: Cryptographic nonce
    """

    chars = string.digits + string.ascii_letters
    nonce = "".join(secrets.choice(chars) for i in range(25))
    LOGGER.debug(f"nonce created: {nonce}")

    return nonce


class Client:
    """
    Provides access to the Onshape REST API.

    Attributes:
        - env (str, default='./.env'): Location of the environment file
        - logging (bool, default=True): Turn logging on or off
    """

    def __init__(self, env: str = "./.env", log_file: str = "./onshape.log", log_level: int = 1) -> None:
        """
        Instantiates an instance of the Onshape class. Reads credentials from a .env file.

        The .env file should be stored in the root project folder; optionally,
        you can specify the location of a different file.

        Args:
            - env (str, default='./.env'): Environment file location
        """

        self._url: str = BASE_URL
        self._access_key, self._secret_key = load_env_variables(env)
        LOGGER.set_file_name(log_file)
        LOGGER.set_stream_level(LOG_LEVEL[log_level])
        LOGGER.info(f"Onshape API initialized with env file: {env}")

    def get_document(self, did: str) -> DocumentMetaData:
        """
        Get details for a specified document.

        Args:
            - did (str): Document ID

        Returns:
            - requests.Response: Onshape response data
        """
        _request_json = self.request(HTTP.GET, "/api/documents/" + did).json()

        return DocumentMetaData.model_validate(_request_json)

    def get_elements(self, did: str, wtype: str, wid: str) -> dict[str, Element]:
        """
        Get list of elements in a document.

        Args:
            - did (str): Document ID
            - wtype (str): Workspace type (w, v, or m)
            - wid (str): Workspace ID

        Returns:
            -
        """

        # /documents/d/{did}/{wvm}/{wvmid}/elements
        _request_path = "/api/documents/d/" + did + "/" + wtype + "/" + wid + "/elements"
        _elements_json = self.request(
            HTTP.GET,
            _request_path,
        ).json()

        return {element["name"]: Element.model_validate(element) for element in _elements_json}

    def get_features_from_partstudio(self, did: str, wid: str, eid: str) -> requests.Response:
        """
        Gets the feature list for specified document / workspace / part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        """

        return self.request(
            HTTP.GET,
            "/api/partstudios/d/" + did + "/w/" + wid + "/e/" + eid + "/features",
        )

    def get_features_from_assembly(self, did: str, wtype: str, wid: str, eid: str) -> dict[str, Any]:
        """
        Gets the feature list for specified document / workspace / part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - json: Onshape response data
        """

        return self.request(
            "get", "/api/assemblies/d/" + did + "/" + wtype + "/" + wid + "/e/" + eid + "/features"
        ).json()

    def get_variables(self, did: str, wid: str, eid: str) -> dict[str, Variable]:
        """
        Get list of variables in a variable studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        """
        _request_path = "/api/variables/d/" + did + "/w/" + wid + "/e/" + eid + "/variables"

        _variables_json = self.request(
            HTTP.GET,
            _request_path,
        ).json()

        return {variable["name"]: Variable.model_validate(variable) for variable in _variables_json[0]["variables"]}

    def set_variables(self, did: str, wid: str, eid: str, variables: dict[str, Variable]) -> requests.Response:
        """
        Set variables in a variable studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID
            - variables (dict): Dictionary of variable name and value pairs

        Returns:
            - requests.Response: Onshape response data
        """

        payload = [variable.model_dump() for variable in variables.values()]

        # api/v9/variables/d/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/cba5e3ca026547f34f8d9f0f/variables
        _request_path = "/api/variables/d/" + did + "/w/" + wid + "/e/" + eid + "/variables"

        return self.request(
            HTTP.POST,
            _request_path,
            body=payload,
        )

    def create_assembly(self, did: str, wid: str, name: str = "My Assembly") -> requests.Response:
        """
        Creates a new assembly element in the specified document / workspace.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - name (str, default='My Assembly')

        Returns:
            - requests.Response: Onshape response data
        """

        payload = {"name": name}

        return self.request(HTTP.POST, "/api/assemblies/d/" + did + "/w/" + wid, body=payload)

    def get_assembly(
        self, did: str, wtype: str, wid: str, eid: str, configuration: str = "default"
    ) -> tuple[Assembly, dict[str, Any]]:
        _request_path = "/api/assemblies/d/" + did + "/" + wtype + "/" + wid + "/e/" + eid
        _assembly_json = self.request(
            HTTP.GET,
            _request_path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
        ).json()

        return Assembly.model_validate(_assembly_json), _assembly_json

    def get_parts(self, did: str, wid: str, eid: str) -> None:
        pass

    def download_stl(self, did: str, wid: str, eid: str, partID: str, buffer: BinaryIO) -> None:
        """
        Exports STL export from a part studio and saves it to a file.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID
            - partID (str): Part ID
            - save_path (str): Path to save the STL file

        Returns:
            - str: Path to the saved STL file or an error message
        """

        req_headers = {"Accept": "application/vnd.onshape.v1+octet-stream"}
        _request_path = f"/api/parts/d/{did}/w/{wid}/e/{eid}/partid/{partID}/stl"
        _query = {
            "mode": "binary",
            "grouping": True,
            "units": "meter",
        }
        response = self.request(
            HTTP.GET,
            path=_request_path,
            headers=req_headers,
            query=_query,
            log_response=False,
        )
        if response.status_code == 200:
            buffer.write(response.content)
        else:
            LOGGER.info(f"Failed to download STL file: {response.status_code} - {response.text}")

    def get_mass_properties(self, did: str, wid: str, eid: str, partID: str) -> MassModel:
        """
        Get mass properties for a part in a part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID
            - partID (str): Part ID

        Returns:
            - requests.Response: Onshape response data
        """
        _request_path = "/api/parts/d/" + did + "/w/" + wid + "/e/" + eid + "/partid/" + partID + "/massproperties"
        _resonse_json = self.request(HTTP.GET, _request_path, {"useMassPropertyOverrides": True}).json()

        return MassModel.model_validate(_resonse_json["bodies"][partID])

    def request(
        self,
        method: Union[HTTP, str],
        path: str,
        query: Optional[dict[str, Any]] = None,
        headers: Optional[dict[str, str]] = None,
        body: Optional[dict[str, Any]] = None,
        base_url: Optional[str] = None,
        log_response: bool = True,
    ) -> requests.Response:
        """
        Issues a request to Onshape

        Args:
            - method (str): HTTP method
            - path (str): Path  e.g. /api/documents/:id
            - query (dict, default={}): Query params in key-value pairs
            - headers (dict, default={}): Key-value pairs of headers
            - body (dict, default={}): Body for POST request
            - base_url (str, default=None): Host, including scheme and port (if different from keys file)

        Returns:
            - requests.Response: Object containing the response from Onshape
        """
        if query is None:
            query = {}
        if headers is None:
            headers = {}
        # if body is None:
        #     body = {}
        if base_url is None:
            base_url = self._url

        req_headers = self._make_headers(method, path, query, headers)
        url = self._build_url(base_url, path, query)

        LOGGER.debug(f"Request body: {body}")
        LOGGER.debug(f"Request headers: {req_headers}")
        LOGGER.debug(f"Request URL: {url}")

        res = self._send_request(method, url, req_headers, body)

        if res.status_code == 307:
            return self._handle_redirect(res, method, headers, log_response)
        else:
            if log_response:
                self._log_response(res)

        return res

    def _build_url(self, base_url: str, path: str, query: dict[str, Any]) -> str:
        return base_url + path + "?" + urlencode(query)

    def _send_request(
        self, method: Union[HTTP, str], url: str, headers: dict[str, str], body: Optional[dict[str, Any]]
    ) -> requests.Response:
        return requests.request(
            method,
            url,
            headers=headers,
            json=body,
            allow_redirects=False,
            stream=True,
            timeout=10,  # Specify an appropriate timeout value in seconds
        )

    def _handle_redirect(
        self, res: requests.Response, method: Union[HTTP, str], headers: dict[str, str], log_response: bool = True
    ) -> requests.Response:
        location = urlparse(res.headers["Location"])
        querystring = parse_qs(location.query)

        LOGGER.debug(f"Request redirected to: {location.geturl()}")

        new_query = {key: querystring[key][0] for key in querystring}
        new_base_url = location.scheme + "://" + location.netloc

        return self.request(
            method, location.path, query=new_query, headers=headers, base_url=new_base_url, log_response=log_response
        )

    def _log_response(self, res: requests.Response) -> None:
        if not 200 <= res.status_code <= 206:
            LOGGER.debug(f"Request failed, details: {res.text}")
        else:
            LOGGER.debug(f"Request succeeded, details: {res.text}")

    def _make_auth(
        self,
        method: Union[HTTP, str],
        date: str,
        nonce: str,
        path: str,
        query: Optional[dict[str, Any]] = None,
        ctype: str = "application/json",
    ) -> str:
        """
        Create the request signature to authenticate

        Args:
            - method (str): HTTP method
            - date (str): HTTP date header string
            - nonce (str): Cryptographic nonce
            - path (str): URL pathname
            - query (dict, default={}): URL query string in key-value pairs
            - ctype (str, default='application/json'): HTTP Content-Type
        """

        if query is None:
            query = {}
        query = urlencode(query)

        hmac_str = (
            (method + "\n" + nonce + "\n" + date + "\n" + ctype + "\n" + path + "\n" + query + "\n")
            .lower()
            .encode("utf-8")
        )

        signature = base64.b64encode(
            hmac.new(self._secret_key.encode("utf-8"), hmac_str, digestmod=hashlib.sha256).digest()
        )
        auth = "On " + self._access_key + ":HmacSHA256:" + signature.decode("utf-8")

        LOGGER.debug(f"query: {query}, hmac_str: {hmac_str}, signature: {signature}, auth: {auth}")

        return auth

    def _make_headers(
        self,
        method: Union[HTTP, str],
        path: str,
        query: Optional[dict[str, Any]] = None,
        headers: Optional[dict[str, str]] = None,
    ) -> dict[str, str]:
        """
        Creates a headers object to sign the request

        Args:
            - method (str): HTTP method
            - path (str): Request path, e.g. /api/documents. No query string
            - query (dict, default={}): Query string in key-value format
            - headers (dict, default={}): Other headers to pass in

        Returns:
            - dict: Dictionary containing all headers
        """

        if headers is None:
            headers = {}
        if query is None:
            query = {}
        date = datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S GMT")
        nonce = make_nonce()
        ctype = headers.get("Content-Type") if headers.get("Content-Type") else "application/json"

        auth = self._make_auth(method, date, nonce, path, query=query, ctype=ctype)

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
