from enum import Enum
from typing import Union

import numpy as np
from pydantic import BaseModel, field_validator

from onshape_api.utilities.helpers import generate_uid


class InstanceType(str, Enum):
    PART = "Part"
    ASSEMBLY = "Assembly"


class MATETYPE(str, Enum):
    SLIDER = "SLIDER"
    CYLINDRICAL = "CYLINDRICAL"
    REVOLUTE = "REVOLUTE"
    PIN_SLOT = "PIN_SLOT"
    PLANAR = "PLANAR"
    BALL = "BALL"
    FASTENED = "FASTENED"
    PARALLEL = "PARALLEL"


class AssemblyFeatureType(str, Enum):
    MATE = "mate"


class Occurrence(BaseModel):
    """
    Occurence model

    {
    "fixed" : false,
    "transform" :
        [ 0.8660254037844396, 0.0, 0.5000000000000004, 0.09583333333333346,
        0.0, 1.0, 0.0, -1.53080849893419E-19,
        -0.5000000000000004, 0.0, 0.8660254037844396, 0.16598820239201767,
        0.0, 0.0, 0.0, 1.0 ],
    "hidden" : false,
    "path" : [ "M0Cyvy+yIq8Rd7En0" ]
    }
    """

    fixed: bool
    transform: list[float]
    hidden: bool
    path: list[str]

    @field_validator("transform")
    def check_transform(cls, v: list[float]) -> list[float]:
        if len(v) != 16:
            raise ValueError("Transform must have 16 values")

        return v


class IDBase(BaseModel):
    """
    Base model for Part in Assembly context
    {
        "fullConfiguration" : "default",
        "configuration" : "default",
        "documentId" : "a1c1addf75444f54b504f25c",
        "elementId" : "0b0c209535554345432581fe",
        "documentMicroversion" : "12fabf866bef5a9114d8c4d2"
    }
    """

    fullConfiguration: str
    configuration: str
    documentId: str
    elementId: str
    documentMicroversion: str

    @field_validator("documentId", "elementId", "documentMicroversion")
    def check_ids(cls, v: str) -> str:
        if len(v) != 24:
            raise ValueError("DocumentId must have 24 characters")

        return v

    @property
    def uid(self) -> str:
        return generate_uid([self.documentId, self.documentMicroversion, self.elementId, self.fullConfiguration])


class Part(IDBase):
    """
    Part data model
    {
      "isStandardContent" : false,
      "partId" : "RDBD",
      "bodyType" : "solid",

      "fullConfiguration" : "default",
      "configuration" : "default",
      "documentId" : "a1c1addf75444f54b504f25c",
      "elementId" : "0b0c209535554345432581fe",
      "documentMicroversion" : "349f6413cafefe8fb4ab3b07"
    }
    """

    isStandardContent: bool
    partId: str
    bodyType: str

    @property
    def uid(self) -> str:
        return generate_uid([
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        ])


class PartInstance(IDBase):
    """
    Part Instance model
    {
        "isStandardContent" : false,
        "type" : "Part",
        "id" : "M0Cyvy+yIq8Rd7En0",
        "name" : "Part 1 <2>",
        "suppressed" : false,
        "partId" : "JHD",

        "fullConfiguration" : "default",
        "configuration" : "default",
        "documentId" : "a1c1addf75444f54b504f25c",
        "elementId" : "a86aaf34d2f4353288df8812",
        "documentMicroversion" : "12fabf866bef5a9114d8c4d2"
    }
    """

    isStandardContent: bool
    type: InstanceType
    id: str
    name: str
    suppressed: bool
    partId: str

    @field_validator("type")
    def check_type(cls, v: InstanceType) -> InstanceType:
        if v != InstanceType.PART:
            raise ValueError("Type must be Part")

        return v

    @property
    def uid(self) -> str:
        return generate_uid([
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        ])


class AssemblyInstance(IDBase):
    """
    Assembly Instance model
    {
        "id" : "Mon18P7LPP8A9STk+",
        "type" : "Assembly",
        "name" : "subAssembly <1>",
        "suppressed" : false,

        "fullConfiguration" : "default",
        "configuration" : "default",
        "documentId" : "a1c1addf75444f54b504f25c",
        "elementId" : "f0b3a4afab120f778a4037df",
        "documentMicroversion" : "349f6413cafefe8fb4ab3b07"
    }
    """

    id: str
    type: InstanceType
    name: str
    suppressed: bool

    @field_validator("type")
    def check_type(cls, v: InstanceType) -> InstanceType:
        if v != InstanceType.ASSEMBLY:
            raise ValueError("Type must be Assembly")

        return v


Instance = Union[PartInstance, AssemblyInstance]


class MatedCS(BaseModel):
    """
    Mated CS model
    {
        "xAxis" : [ 1.0, 0.0, 0.0 ],
        "yAxis" : [ 0.0, 0.0, -1.0 ],
        "zAxis" : [ 0.0, 1.0, 0.0 ],
        "origin" : [ 0.0, -0.0505, 0.0 ]
    }
    """

    xAxis: list[float]
    yAxis: list[float]
    zAxis: list[float]
    origin: list[float]

    @field_validator("xAxis", "yAxis", "zAxis", "origin")
    def check_vectors(cls, v: list[float]) -> list[float]:
        if len(v) != 3:
            raise ValueError("Vectors must have 3 values")

        return v

    @property
    def part_to_mate_tf(self) -> np.matrix:
        rotation_matrix = np.array([self.xAxis, self.yAxis, self.zAxis]).T
        translation_vector = np.array(self.origin)
        part_to_mate_tf = np.eye(4)
        part_to_mate_tf[:3, :3] = rotation_matrix
        part_to_mate_tf[:3, 3] = translation_vector
        return np.matrix(part_to_mate_tf)


class MatedEntity(BaseModel):
    """
    MatedEntity data model
    {
        "matedOccurrence" : [ "MDUJyqGNo7JJll+/h" ],
        "matedCS" :
        {
            "xAxis" : [ 1.0, 0.0, 0.0 ],
            "yAxis" : [ 0.0, 0.0, -1.0 ],
            "zAxis" : [ 0.0, 1.0, 0.0 ],
            "origin" : [ 0.0, -0.0505, 0.0 ]
        }
    }
    """

    matedOccurrence: list[str]
    matedCS: MatedCS


class MateFeatureData(BaseModel):
    """
    MateFeatureData data model
    {
        "matedEntities" :
        [
            {
                "matedOccurrence" : [ "MDUJyqGNo7JJll+/h" ],
                "matedCS" :
                {
                    "xAxis" : [ 1.0, 0.0, 0.0 ],
                    "yAxis" : [ 0.0, 0.0, -1.0 ],
                    "zAxis" : [ 0.0, 1.0, 0.0 ],
                    "origin" : [ 0.0, -0.0505, 0.0 ]
                }
            }, {
                "matedOccurrence" : [ "MwoBIsds8rn1/0QXA" ],
                "matedCS" :
                {
                    "xAxis" : [ 0.8660254037844387, 0.0, -0.49999999999999994 ],
                    "yAxis" : [ -0.49999999999999994, 0.0, -0.8660254037844387 ],
                    "zAxis" : [ 0.0, 1.0, 0.0 ],
                    "origin" : [ 0.0, -0.0505, 0.0 ]
                }
            }
        ],
        "mateType" : "FASTENED",
        "name" : "Fastened 1"
    }
    """

    matedEntities: list[MatedEntity]
    mateType: MATETYPE
    name: str


class MateFeature(BaseModel):
    """
    Feature model
    {
        "id" : "M11CFUi4PcoWOBxpJ",
        "suppressed" : false,
        "featureType" : "mate",
        "featureData" :
        {
          "matedEntities" :
            [
                {
                    "matedOccurrence" : [ "MDUJyqGNo7JJll+/h" ],
                    "matedCS" :
                    {
                        "xAxis" : [ 1.0, 0.0, 0.0 ],
                        "yAxis" : [ 0.0, 0.0, -1.0 ],
                        "zAxis" : [ 0.0, 1.0, 0.0 ],
                        "origin" : [ 0.0, -0.0505, 0.0 ]
                    }
                }, {
                    "matedOccurrence" : [ "MwoBIsds8rn1/0QXA" ],
                    "matedCS" :
                    {
                        "xAxis" : [ 0.8660254037844387, 0.0, -0.49999999999999994 ],
                        "yAxis" : [ -0.49999999999999994, 0.0, -0.8660254037844387 ],
                        "zAxis" : [ 0.0, 1.0, 0.0 ],
                        "origin" : [ 0.0, -0.0505, 0.0 ]
                    }
                }
            ],
          "mateType" : "FASTENED",
          "name" : "Fastened 1"
        }
    }
    """

    id: str
    suppressed: bool
    featureType: str
    featureData: MateFeatureData

    # @field_validator("id")
    # def check_id(cls, v):
    #     if len(v) != 17:
    #         raise ValueError("Id must have 17 characters")

    #     return v

    @field_validator("featureType")
    def check_featureType(cls, v: str) -> str:
        if v != AssemblyFeatureType.MATE:
            raise ValueError("FeatureType must be Mate")

        return v


class SubAssembly(IDBase):
    """
    SubAssembly data model
    """

    instances: list[Union[PartInstance, AssemblyInstance]]
    patterns: list[dict]
    features: list[MateFeature]

    @property
    def uid(self) -> str:
        return generate_uid([self.documentId, self.documentMicroversion, self.elementId, self.fullConfiguration])


class RootAssembly(SubAssembly):
    """
    RootAssembly data model
    """

    occurrences: list[Occurrence]


class Assembly(BaseModel):
    """
    Assembly data model
    """

    rootAssembly: RootAssembly
    subAssemblies: list[SubAssembly]
    parts: list[Part]
    partStudioFeatures: list[dict]
