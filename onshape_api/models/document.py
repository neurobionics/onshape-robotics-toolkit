from enum import Enum
from typing import Union, cast, Tuple

import regex as re
from pydantic import BaseModel, field_validator

__all__ = ["WORKSPACE_TYPE", "Document", "parse_url"]


class WORKSPACE_TYPE(str, Enum):
    W = "w"
    V = "v"
    M = "m"


DOCUMENT_PATTERN = r"https://cad.onshape.com/documents/([\w\d]+)/(w|v|m)/([\w\d]+)/e/([\w\d]+)"


def parse_url(url: str) -> Tuple[str, WORKSPACE_TYPE, str, str]:
    pattern = re.match(
        DOCUMENT_PATTERN,
        url,
    )

    if not pattern:
        raise ValueError("Invalid Onshape URL")

    did = pattern.group(1)
    wtype = cast(WORKSPACE_TYPE, pattern.group(2))
    wid = pattern.group(3)
    eid = pattern.group(4)

    return did, wtype, wid, eid


class Document(BaseModel):
    url: Union[str, None]
    did: str
    wtype: WORKSPACE_TYPE
    wid: str
    eid: str

    @field_validator("did", "wid", "eid")
    def check_ids(cls, value: str) -> str:
        if not value:
            raise ValueError("ID cannot be empty, please check the URL")
        if not len(value) == 24:
            raise ValueError("ID must be 24 characters long, please check the URL")
        return value

    @field_validator("wtype")
    def check_wtype(cls, value: WORKSPACE_TYPE) -> WORKSPACE_TYPE:
        if not value:
            raise ValueError("Workspace type cannot be empty, please check the URL")

        if value not in WORKSPACE_TYPE.__members__.values():
            raise ValueError(
                f"Invalid workspace type. Must be one of {WORKSPACE_TYPE.__members__.values()}, please check the URL"
            )

        return value

    @classmethod
    def from_url(cls, url: str) -> "Document":
        did, wtype, wid, eid = parse_url(url)
        return cls(url=url, did=did, wtype=wtype, wid=wid, eid=eid)


class DefaultWorkspace(BaseModel):
    id: str


class DocumentMetaData(BaseModel):
    defaultWorkspace: DefaultWorkspace
    name: str


if __name__ == "__main__":
    doc = Document(
        url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
    )
    print(doc.did)
