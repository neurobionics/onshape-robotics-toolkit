"""
This module defines data models for Assembly entities retrieved from Onshape REST API responses.

The data models are implemented as Pydantic BaseModel classes, which are used to

    1. Parse JSON responses from the API into Python objects.
    2. Validate the structure and types of the JSON responses.
    3. Provide type hints for better code clarity and autocompletion.

These models ensure that the data received from the API adheres to the expected format and types, facilitating easier
and safer manipulation of the data within the application.

Models:
    - **Occurrence**: Represents an occurrence of a part or sub-assembly within an assembly.
    - **Part**: Represents a part within an assembly, including its properties and configuration.
    - **PartInstance**: Represents an instance of a part within an assembly.
    - **AssemblyInstance**: Represents an instance of an assembly within another assembly.
    - **AssemblyFeature**: Represents a feature within an assembly, such as a mate or pattern.
    - **Pattern**: Represents a pattern feature within an assembly, defining repeated instances of
      parts or sub-assemblies.
    - **SubAssembly**: Represents a sub-assembly within a larger assembly.
    - **RootAssembly**: Represents the root assembly, which is the top-level assembly containing all parts
      and sub-assemblies.
    - **Assembly**: Represents the overall assembly, including all parts, sub-assemblies, and features.

Supplementary models:
    - **IDBase**: Base model providing common attributes for Part, SubAssembly, and AssemblyInstance models.
    - **MatedCS**: Represents a coordinate system used for mating parts within an assembly.
    - **MatedEntity**: Represents an entity that is mated within an assembly, including its coordinate system.
    - **MateRelationMate**: Represents a mate relation within an assembly, defining how parts or sub-assemblies
      are connected.
    - **MateGroupFeatureOccurrence**: Represents an occurrence of a mate group feature within an assembly.
    - **MateGroupFeatureData**: Represents data for a mate group feature within an assembly.
    - **MateConnectorFeatureData**: Represents data for a mate connector feature within an assembly.
    - **MateRelationFeatureData**: Represents data for a mate relation feature within an assembly.
    - **MateFeatureData**: Represents data for a mate feature within an assembly.

Enum:
    - **InstanceType**: Enumerates the types of instances in an assembly, e.g. PART, ASSEMBLY.
    - **MateType**: Enumerates the type of mate between two parts or assemblies, e.g. SLIDER,
      CYLINDRICAL, REVOLUTE, etc.
    - **RelationType**: Enumerates the type of mate relation between two parts or assemblies, e.g. LINEAR,
      GEAR, SCREW, etc.
    - **AssemblyFeatureType**: Enumerates the type of assembly feature, e.g. mate, mateRelation,
      mateGroup, mateConnector

"""

from enum import Enum
from typing import Any, Literal, Union

import numpy as np
import numpy.typing as npt
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from onshape_robotics_toolkit.models.document import Document, DocumentMetaData
from onshape_robotics_toolkit.models.mass import MassProperties
from onshape_robotics_toolkit.utilities.helpers import generate_uid


class InstanceType(str, Enum):
    """
    Enumerates the types of instances in an assembly, e.g. PART, ASSEMBLY.

    Attributes:
        PART (str): Represents a part instance.
        ASSEMBLY (str): Represents an assembly instance.

    Examples:
        >>> InstanceType.PART
        'Part'
        >>> InstanceType.ASSEMBLY
        'Assembly'
    """

    PART = "Part"
    ASSEMBLY = "Assembly"


class MateType(str, Enum):
    """
    Enumerates the type of mate between two parts or assemblies, e.g. SLIDER, CYLINDRICAL, REVOLUTE, etc.

    Attributes:
        SLIDER (str): Represents a slider mate.
        CYLINDRICAL (str): Represents a cylindrical mate.
        REVOLUTE (str): Represents a revolute mate.
        PIN_SLOT (str): Represents a pin-slot mate.
        PLANAR (str): Represents a planar mate.
        BALL (str): Represents a ball mate.
        FASTENED (str): Represents a fastened mate.
        PARALLEL (str): Represents a parallel mate.

    Examples:
        >>> MateType.SLIDER
        'SLIDER'
        >>> MateType.CYLINDRICAL
        'CYLINDRICAL'
    """

    SLIDER = "SLIDER"
    CYLINDRICAL = "CYLINDRICAL"
    REVOLUTE = "REVOLUTE"
    PIN_SLOT = "PIN_SLOT"
    PLANAR = "PLANAR"
    BALL = "BALL"
    FASTENED = "FASTENED"
    PARALLEL = "PARALLEL"


class RelationType(str, Enum):
    """
    Enumerates the type of mate relation between two parts or assemblies, e.g. LINEAR, GEAR, SCREW, etc.

    Attributes:
        LINEAR (str): Represents a linear relation.
        GEAR (str): Represents a gear relation.
        SCREW (str): Represents a screw relation.
        RACK_AND_PINION (str): Represents a rack and pinion relation.

    Examples:
        >>> RelationType.LINEAR
        'LINEAR'
        >>> RelationType.GEAR
        'GEAR'
    """

    LINEAR = "LINEAR"
    GEAR = "GEAR"
    SCREW = "SCREW"
    RACK_AND_PINION = "RACK_AND_PINION"


class AssemblyFeatureType(str, Enum):
    """
    Enumerates the type of assembly feature, e.g. mate, mateRelation, mateGroup, mateConnector

    Attributes:
        MATE (str): Represents a mate feature.
        MATERELATION (str): Represents a mate relation feature.
        MATEGROUP (str): Represents a mate group feature.
        MATECONNECTOR (str): Represents a mate connector feature.

    Examples:
        >>> AssemblyFeatureType.MATE
        'mate'
        >>> AssemblyFeatureType.MATERELATION
        'mateRelation'
    """

    MATE = "mate"
    MATERELATION = "mateRelation"
    MATEGROUP = "mateGroup"
    MATECONNECTOR = "mateConnector"


class Occurrence(BaseModel):
    """
    Represents an occurrence of a part or sub-assembly within an assembly.

    JSON:
        ```json
            {
                "fixed": false,
                "transform": [
                    0.8660254037844396, 0.0, 0.5000000000000004, 0.09583333333333346,
                    0.0, 1.0, 0.0, -1.53080849893419E-19,
                    -0.5000000000000004, 0.0, 0.8660254037844396, 0.16598820239201767,
                    0.0, 0.0, 0.0, 1.0
                ],
                "hidden": false,
                "path": ["M0Cyvy+yIq8Rd7En0"]
            }
        ```

    Attributes:
        fixed (bool): Indicates if the occurrence is fixed in space.
        transform (list[float]): A 4x4 transformation matrix represented as a list of 16 floats.
        hidden (bool): Indicates if the occurrence is hidden.
        path (list[str]): A list of strings representing the path to the instance.

    Examples:
        >>> Occurrence(
        ...     fixed=False,
        ...     transform=[
        ...         0.8660254037844396, 0.0, 0.5000000000000004, 0.09583333333333346,
        ...         0.0, 1.0, 0.0, -1.53080849893419E-19,
        ...         -0.5000000000000004, 0.0, 0.8660254037844396, 0.16598820239201767,
        ...         0.0, 0.0, 0.0, 1.0)
        ...     ],
        ...     hidden=False,
        ...     path=["M0Cyvy+yIq8Rd7En0"]
        ... )
        Occurrence(
            fixed=False,
            transform=[...],
            hidden=False,
            path=["M0Cyvy+yIq8Rd7En0"]
        )
    """

    fixed: bool = Field(..., description="Indicates if the occurrence is fixed in space.")
    transform: list[float] = Field(..., description="A 4x4 transformation matrix represented as a list of 16 floats.")
    hidden: bool = Field(..., description="Indicates if the occurrence is hidden.")
    path: list[str] = Field(..., description="A list of strings representing the path to the instance.")

    @field_validator("transform")
    def check_transform(cls, v: list[float]) -> list[float]:
        """
        Validates that the transform list has exactly 16 values.

        Args:
            v (list[float]): The transform list to validate.

        Returns:
            list[float]: The validated transform list.

        Raises:
            ValueError: If the transform list does not contain exactly 16 values.
        """
        if len(v) != 16:
            raise ValueError("Transform must have 16 values")

        return v

    @property
    def tf(self) -> npt.NDArray[np.floating[Any]]:
        """
        Converts the flat list representation of the transformation matrix into a 4x4 numpy array.

        Returns:
            np.ndarray: A 4x4 numpy array representing the transformation matrix.
        """
        return np.array(self.transform).reshape(4, 4)

    def tf_wrt(self, ref_tf: np.ndarray) -> npt.NDArray[np.floating[Any]]:
        """
        Computes the transformation matrix of this occurrence with respect to a reference transformation matrix.

        Args:
            ref_tf (np.ndarray): The reference 4x4 transformation matrix.

        Returns:
            np.ndarray: The 4x4 transformation matrix of this occurrence with respect to the reference.
        """
        result: npt.NDArray[np.floating[Any]] = np.linalg.inv(ref_tf) @ self.tf
        return result


class IDBase(BaseModel):
    """
    Base model providing common attributes for Part, SubAssembly, and AssemblyInstance models.

    JSON:
        ```json
            {
                "fullConfiguration" : "default",
                "configuration" : "default",
                "documentId" : "a1c1addf75444f54b504f25c",
                "elementId" : "0b0c209535554345432581fe",
                "documentMicroversion" : "12fabf866bef5a9114d8c4d2"
            }
        ```

    Attributes:
        fullConfiguration (str): The full configuration of the entity.
        configuration (str): The configuration of the entity.
        documentId (str): The unique identifier of the entity.
        elementId (str): The unique identifier of the entity.
        documentMicroversion (str): The microversion of the document.
    """

    fullConfiguration: str = Field(..., description="The full configuration of the entity.")
    configuration: str = Field(..., description="The configuration of the entity.")
    documentId: str = Field(..., description="The unique identifier of the entity.")
    elementId: str = Field(..., description="The unique identifier of the entity.")
    documentMicroversion: str = Field(..., description="The microversion of the document.")

    @field_validator("documentId", "elementId", "documentMicroversion")
    def check_ids(cls, v: str) -> str:
        """
        Validates that the ID fields have exactly 24 characters.

        Args:
            v (str): The ID field to validate.

        Returns:
            str: The validated ID field.

        Raises:
            ValueError: If the ID field does not contain exactly 24 characters.
        """
        if len(v) != 24:
            raise ValueError("DocumentId must have 24 characters")

        return v

    @property
    def uid(self) -> str:
        """
        Generates a unique identifier for the part.

        Returns:
            str: The unique identifier generated from documentId, documentMicroversion,
                elementId, and fullConfiguration.
        """
        return generate_uid([self.documentId, self.documentMicroversion, self.elementId, self.fullConfiguration])


class PartMetadata(BaseModel):
    pass


class PartMateConnector(BaseModel):
    """
    Represents a mate connector within a part.

    JSON:
        ```json
            {
            "mateConnectorCS" : {
                "xAxis" : [ 1.0, 0.0, 0.0 ],
                "yAxis" : [ 0.0, 1.0, 0.0 ],
                "zAxis" : [ 0.0, 0.0, 1.0 ],
                "origin" : [ 0.0, 0.0, 0.024999999999999984 ]
            },
            "featureId" : "FuB5m1oLMD3WyJ1_1"
            }
        ```

    Attributes:
        mateConnectorCS (MatedCS): The coordinate system used for the mate connector.
        featureId (str): The unique identifier of the mate connector feature.

    Examples:
        >>> PartMateConnector(
        ...     mateConnectorCS=MatedCS(
        ...         xAxis=[1.0, 0.0, 0.0],
        ...         yAxis=[0.0, 1.0, 0.0],
        ...         zAxis=[0.0, 0.0, 1.0],
        ...         origin=[0.0, 0.0, 0.024999999999999984],
        ...     ),
        ...     featureId="FuB5m1oLMD3WyJ1_1",
        ... )
        PartMateConnector(
            mateConnectorCS=MatedCS(
                xAxis=[1.0, 0.0, 0.0],
                yAxis=[0.0, 1.0, 0.0],
                zAxis=[0.0, 0.0, 1.0],
                origin=[0.0, 0.0, 0.024999999999999984]
            ),
            featureId="FuB5m1oLMD3WyJ1_1"
        )
    """

    mateConnectorCS: "MatedCS" = Field(..., description="The coordinate system used for the mate connector.")
    featureId: str = Field(..., description="The unique identifier of the mate connector feature.")


class Part(IDBase):
    """
    Represents a part within an assembly, including its properties and configuration.

    JSON:
        ```json
            {
                "isStandardContent": false,
                "partId": "RDBD",
                "bodyType": "solid",
                "fullConfiguration": "default",
                "configuration": "default",
                "documentId": "a1c1addf75444f54b504f25c",
                "elementId": "0b0c209535554345432581fe",
                "documentMicroversion": "349f6413cafefe8fb4ab3b07"
            }
        ```

    Attributes:
        isStandardContent (bool): Indicates if the part is standard content.
        partId (str): The unique identifier of the part.
        bodyType (str): The type of the body (e.g., solid, surface).

    Custom Attributes:
        MassProperty (Union[MassProperties, None]): The mass properties of the part, if available.

    Examples:
        >>> Part(
        ...     isStandardContent=False,
        ...     partId="RDBD",
        ...     bodyType="solid",
        ... )
        Part(
            isStandardContent=False,
            partId="RDBD",
            bodyType="solid",
            MassProperty=None
        )

    """

    isStandardContent: bool = Field(..., description="Indicates if the part is standard content.")
    partId: str = Field(..., description="The unique identifier of the part.")
    bodyType: str = Field(..., description="The type of the body (e.g., solid, surface).")
    mateConnectors: list[Union[PartMateConnector, None]] = Field(
        default_factory=list,
        description="The mate connectors that belong to the part.",
    )
    documentVersion: Union[str, None] = Field(None, description="The version of the document.")
    MassProperty: Union[MassProperties, None] = Field(
        None, description="The mass properties of the part, this is a retrieved via a separate API call."
    )

    isRigidAssembly: bool = Field(
        False, description="Indicates if the part is a rigid assembly, i.e., a sub-assembly with no degrees of freedom."
    )
    rigidAssemblyKey: Union[Any, None] = Field(
        None, description="The unique key of the rigid assembly, if it is a sub-assembly."
    )
    rigidAssemblyToPartTF: Union["MatedCS", None] = Field(
        None, description="The transformation matrix from the rigid assembly to the part coordinate system."
    )
    rigidAssemblyWorkspaceId: Union[str, None] = Field(
        None, description="The workspace ID of the rigid assembly, if it is a sub-assembly."
    )
    worldToPartTF: Union["MatedCS", None] = Field(
        None, description="The transformation matrix representing the part wrt world coordinates"
    )

    @property
    def uid(self) -> str:
        """
        Generates a unique identifier for the part.

        Returns:
            str: The unique identifier generated from documentId, documentMicroversion,
                elementId, partId, and fullConfiguration.
        """
        return generate_uid([
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        ])


class PartInstance(IDBase):
    """
    Represents an instance of a part within an assembly.

    JSON:
        ```json
            {
                "isStandardContent": false,
                "type": "Part",
                "id": "M0Cyvy+yIq8Rd7En0",
                "name": "Part 1 <2>",
                "suppressed": false,
                "partId": "JHD",
                "fullConfiguration": "default",
                "configuration": "default",
                "documentId": "a1c1addf75444f54b504f25c",
                "elementId": "0b0c209535554345432581fe",
                "documentMicroversion": "349f6413cafefe8fb4ab3b07"
            }
        ```

    Attributes:
        isStandardContent (bool): Indicates if the part is standard content.
        type (InstanceType): The type of the instance, must be 'Part'.
        id (str): The unique identifier for the part instance.
        name (str): The name of the part instance.
        suppressed (bool): Indicates if the part instance is suppressed.
        partId (str): The identifier for the part.

    Examples:
        >>> PartInstance(
        ...     isStandardContent=False,
        ...     type=InstanceType.PART,
        ...     id="M0Cyvy+yIq8Rd7En0",
        ...     name="Part 1 <2>",
        ...     suppressed=False,
        ...     partId="JHD",
        ... )
        PartInstance(
            isStandardContent=False,
            type=InstanceType.PART,
            id="M0Cyvy+yIq8Rd7En0",
            name="Part 1 <2>",
            suppressed=False,
            partId="JHD",
        )
    """

    isStandardContent: bool = Field(..., description="Indicates if the part is standard content.")
    type: InstanceType = Field(..., description="The type of the instance, must be 'Part'.")
    documentVersion: Union[str, None] = Field(None, description="The version of the document.")
    id: str = Field(..., description="The unique identifier for the part instance.")
    name: str = Field(..., description="The name of the part instance.")
    suppressed: bool = Field(..., description="Indicates if the part instance is suppressed.")
    partId: str = Field(..., description="The identifier for the part.")

    @field_validator("type")
    def check_type(cls, v: InstanceType) -> InstanceType:
        """
        Validates that the type is 'Part'.

        Args:
            v (InstanceType): The type to validate.

        Returns:
            InstanceType: The validated type.
        """
        if v != InstanceType.PART:
            raise ValueError("Type must be Part")

        return v

    @property
    def uid(self) -> str:
        """
        Generates a unique identifier for the part instance based on its attributes.

        Returns:
            str: The unique identifier for the part instance.
        """
        return generate_uid([
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        ])


class AssemblyInstance(IDBase):
    """
    Represents an instance of an assembly within another assembly.

    JSON:
        ```json
            {
                "id": "Mon18P7LPP8A9STk+",
                "type": "Assembly",
                "name": "subAssembly",
                "suppressed": false,
                "fullConfiguration": "default",
                "configuration": "default",
                "documentId": "a1c1addf75444f54b504f25c",
                "elementId": "f0b3a4afab120f778a4037df",
                "documentMicroversion": "349f6413cafefe8fb4ab3b07"
            }
        ```

    Attributes:
        id (str): The unique identifier for the assembly instance.
        type (InstanceType): The type of the instance, must be 'Assembly'.
        name (str): The name of the assembly instance.
        suppressed (bool): Indicates if the assembly instance is suppressed.

    Examples:
        >>> AssemblyInstance(
        ...     id="Mon18P7LPP8A9STk+",
        ...     type=InstanceType.ASSEMBLY,
        ...     name="subAssembly",
        ...     suppressed=False,
        ... )
        AssemblyInstance(
            id="Mon18P7LPP8A9STk+",
            type=InstanceType.ASSEMBLY,
            name="subAssembly",
            suppressed=False,
        )
    """

    id: str = Field(..., description="The unique identifier for the assembly instance.")
    type: InstanceType = Field(..., description="The type of the instance, must be 'Assembly'.")
    name: str = Field(..., description="The name of the assembly instance.")
    suppressed: bool = Field(..., description="Indicates if the assembly instance is suppressed.")
    documentVersion: Union[str, None] = Field(None, description="The version of the document.")

    isRigid: bool = Field(
        False,
        description="Indicates if the assembly instance is a rigid assembly, i.e., \
        a sub-assembly with no degrees of freedom.",
    )

    @field_validator("type")
    def check_type(cls, v: InstanceType) -> InstanceType:
        """
        Validates that the type is 'Assembly'.

        Args:
            v (InstanceType): The type to validate.

        Returns:
            InstanceType: The validated type.
        """
        if v != InstanceType.ASSEMBLY:
            raise ValueError("Type must be Assembly")

        return v


class MatedCS(BaseModel):
    """
    Represents a coordinate system used for mating parts within an assembly.

    JSON:
        ```json
            {
                "xAxis" : [ 1.0, 0.0, 0.0 ],
                "yAxis" : [ 0.0, 0.0, -1.0 ],
                "zAxis" : [ 0.0, 1.0, 0.0 ],
                "origin" : [ 0.0, -0.0505, 0.0 ]
            }
        ```

    Attributes:
        xAxis (list[float]): The x-axis vector of the coordinate system.
        yAxis (list[float]): The y-axis vector of the coordinate system.
        zAxis (list[float]): The z-axis vector of the coordinate system.
        origin (list[float]): The origin point of the coordinate system.

    Examples:
        >>> MatedCS(
        ...     xAxis=[1.0, 0.0, 0.0],
        ...     yAxis=[0.0, 0.0, -1.0],
        ...     zAxis=[0.0, 1.0, 0.0],
        ...     origin=[0.0, -0.0505, 0.0],
        ... )
        MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 0.0, -1.0],
            zAxis=[0.0, 1.0, 0.0],
            origin=[0.0, -0.0505, 0.0]
        )
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    xAxis: list[float] = Field(..., description="The x-axis vector of the coordinate system.")
    yAxis: list[float] = Field(..., description="The y-axis vector of the coordinate system.")
    zAxis: list[float] = Field(..., description="The z-axis vector of the coordinate system.")
    origin: list[float] = Field(..., description="The origin point of the coordinate system.")

    tf: Union[np.ndarray, None] = Field(
        None, description="The 4x4 transformation matrix from the part coordinate system to the mate coordinate system."
    )

    @field_validator("xAxis", "yAxis", "zAxis", "origin")
    def check_vectors(cls, v: list[float]) -> list[float]:
        """
        Validates that the vectors have exactly 3 values.

        Args:
            v (list[float]): The vector to validate.

        Returns:
            list[float]: The validated vector.

        Raises:
            ValueError: If the vector does not have exactly 3 values.
        """
        if len(v) != 3:
            raise ValueError("Vectors must have 3 values")

        return v

    @property
    def to_tf(self) -> np.ndarray:
        """
        Generates a transformation matrix from the part coordinate system to the mate coordinate system.

        Returns:
            np.ndarray: The 4x4 transformation matrix.
        """
        if self.tf is not None:
            return self.tf

        rotation_matrix = np.array([self.xAxis, self.yAxis, self.zAxis]).T
        translation_vector = np.array(self.origin)
        part_to_mate_tf = np.eye(4)
        part_to_mate_tf[:3, :3] = rotation_matrix
        part_to_mate_tf[:3, 3] = translation_vector
        return part_to_mate_tf

    @classmethod
    def from_tf(cls, tf: np.ndarray) -> "MatedCS":
        """
        Creates a MatedCS object from a 4x4 transformation matrix.

        Args:
            tf (np.ndarray): The 4x4 transformation matrix.

        Returns:
            MatedCS: The MatedCS object created from the transformation matrix.
        """
        # Ensure tf is a regular ndarray
        tf_array = np.asarray(tf)
        return MatedCS(
            xAxis=tf_array[:3, 0].flatten().tolist(),
            yAxis=tf_array[:3, 1].flatten().tolist(),
            zAxis=tf_array[:3, 2].flatten().tolist(),
            origin=tf_array[:3, 3].flatten().tolist(),
            tf=tf_array,
        )


class MatedEntity(BaseModel):
    """
    Represents an entity that is mated within an assembly, including its coordinate system.

    JSON:
        ```json
            {
                "matedOccurrence": ["MDUJyqGNo7JJll+/h"],
                "matedCS": {
                    "xAxis": [1.0, 0.0, 0.0],
                    "yAxis": [0.0, 0.0, -1.0],
                    "zAxis": [0.0, 1.0, 0.0],
                    "origin": [0.0, -0.0505, 0.0]
                }
            }
        ```

    Attributes:
        matedOccurrence (list[str]): A list of identifiers for the occurrences that are mated.
        matedCS (MatedCS): The coordinate system used for mating the parts.

    Examples:
        >>> MatedEntity(
        ...     matedOccurrence=["MDUJyqGNo7JJll+/h"],
        ...     matedCS=MatedCS(
        ...         xAxis=[1.0, 0.0, 0.0],
        ...         yAxis=[0.0, 0.0, -1.0],
        ...         zAxis=[0.0, 1.0, 0.0],
        ...         origin=[0.0, -0.0505, 0.0],
        ...     ),
        ... )
        MatedEntity(
            matedOccurrence=["MDUJyqGNo7JJll+/h"],
            matedCS=MatedCS(
                xAxis=[1.0, 0.0, 0.0],
                yAxis=[0.0, 0.0, -1.0],
                zAxis=[0.0, 1.0, 0.0],
                origin=[0.0, -0.0505, 0.0]
            )
        )
    """

    matedOccurrence: list[str] = Field(..., description="A list of identifiers for the occurrences that are mated.")
    matedCS: MatedCS = Field(..., description="The coordinate system used for mating the parts.")


class MateRelationMate(BaseModel):
    """
    Represents a mate relation within an assembly, defining how parts or sub-assemblies are connected.

    JSON:
        ```json
            {
                "featureId": "S4/TgCRmQt1nIHHp",
                "occurrence": []
            }
        ```

    Attributes:
        featureId (str): The unique identifier of the mate feature.
        occurrence (list[str]): A list of identifiers for the occurrences involved in the mate relation.

    Examples:
        >>> MateRelationMate(
        ...     featureId="S4/TgCRmQt1nIHHp",
        ...     occurrence=[],
        ... )
        MateRelationMate(
            featureId="S4/TgCRmQt1nIHHp",
            occurrence=[],
        )
    """

    featureId: str = Field(..., description="The unique identifier of the mate feature.")
    occurrence: list[str] = Field(
        ..., description="A list of identifiers for the occurrences involved in the mate relation."
    )


class MateGroupFeatureOccurrence(BaseModel):
    """
    Represents an occurrence of a mate group feature within an assembly.

    JSON:
        ```json
            {
                "occurrence": ["MplKLzV/4d+nqmD18"]
            }
        ```

    Attributes:
        occurrence (list[str]): A list of identifiers for the occurrences in the mate group feature.

    Examples:
        >>> MateGroupFeatureOccurrence(
        ...     occurrence=["MplKLzV/4d+nqmD18"],
        ... )
        MateGroupFeatureOccurrence(
            occurrence=["MplKLzV/4d+nqmD18"]
        )
    """

    occurrence: list[str] = Field(
        ..., description="A list of identifiers for the occurrences in the mate group feature."
    )


class MateGroupFeatureData(BaseModel):
    """
    Represents data for a mate group feature within an assembly.

    Mate groups are organizational features in Onshape that group multiple parts together
    without defining kinematic relationships between them. They are used by designers to
    organize complex assemblies and indicate that grouped parts should move as a single
    rigid body.

    - Assemblies containing ONLY mate groups (no regular mates) are treated as rigid bodies
    - Mate groups do not create kinematic joints or edges in the robot graph
    - Parts in mate-group-only assemblies are merged into a single rigid assembly part
    - Mixed assemblies (mate groups + regular mates) remain flexible, with mate groups
      serving as organizational markers only

    JSON:
        ```json
            {
                "occurrences": [
                    {
                        "occurrence": ["MplKLzV/4d+nqmD18"]
                    }
                ],
                "name": "Mate group 1"
            }
        ```

    Attributes:
        occurrences (list[MateGroupFeatureOccurrence]): A list of occurrences in the mate group feature.
        name (str): The name of the mate group feature.

    Custom Attributes:
        id (str): The unique identifier of the feature.

    Examples:
        >>> MateGroupFeatureData(
        ...     occurrences=[
        ...         MateGroupFeatureOccurrence(
        ...             occurrence=["MplKLzV/4d+nqmD18"],
        ...         )
        ...     ],
        ...     name="Mate group 1",
        ... )
        MateGroupFeatureData(
            occurrences=[
                MateGroupFeatureOccurrence(
                    occurrence=["MplKLzV/4d+nqmD18"]
                )
            ],
            name="Mate group 1"
        )
    """

    occurrences: list[MateGroupFeatureOccurrence] = Field(
        ..., description="A list of occurrences in the mate group feature."
    )
    name: str = Field(..., description="The name of the mate group feature.")
    id: str = Field(..., description="The unique identifier of the feature.")


class MateConnectorFeatureData(BaseModel):
    """
    Represents data for a mate connector feature within an assembly.

    JSON:
        ```json
            {
                "mateConnectorCS": {
                    "xAxis": [1.0, 0.0, 0.0],
                    "yAxis": [0.0, 0.0, -1.0],
                    "zAxis": [0.0, 1.0, 0.0],
                    "origin": [0.0, -0.0505, 0.0]
                },
                "occurrence": [
                    "MplKLzV/4d+nqmD18"
                ],
                "name": "Mate connector 1"
            }
        ```

    Attributes:
        mateConnectorCS (MatedCS): The coordinate system used for the mate connector.
        occurrence (list[str]): A list of identifiers for the occurrences involved in the mate connector.
        name (str): The name of the mate connector feature.

    Custom Attributes:
        id (str): The unique identifier of the feature.

    Examples:
        >>> MateConnectorFeatureData(
        ...     mateConnectorCS=MatedCS(
        ...         xAxis=[1.0, 0.0, 0.0],
        ...         yAxis=[0.0, 0.0, -1.0],
        ...         zAxis=[0.0, 1.0, 0.0],
        ...         origin=[0.0, -0.0505, 0.0],
        ...     ),
        ...     occurrence=["MplKLzV/4d+nqmD18"],
        ...     name="Mate connector 1",
        ... )
        MateConnectorFeatureData(
            mateConnectorCS=MatedCS(
                xAxis=[1.0, 0.0, 0.0],
                yAxis=[0.0, 0.0, -1.0],
                zAxis=[0.0, 1.0, 0.0],
                origin=[0.0, -0.0505, 0.0]
            ),
            occurrence=["MplKLzV/4d+nqmD18"],
            name="Mate connector 1"
        )
    """

    mateConnectorCS: MatedCS = Field(..., description="The coordinate system used for the mate connector.")
    occurrence: list[str] = Field(
        ..., description="A list of identifiers for the occurrences involved in the mate connector."
    )
    name: str = Field(..., description="The name of the mate connector feature.")

    id: str = Field(..., description="The unique identifier of the feature.")


class MateRelationFeatureData(BaseModel):
    """
    Represents data for a mate relation feature within an assembly.

    JSON:
        ```json
            {
                "relationType": "GEAR",
                "mates": [
                    {
                    "featureId": "S4/TgCRmQt1nIHHp",
                    "occurrence": []
                    },
                    {
                    "featureId": "QwaoOeXYPifsN7CP",
                    "occurrence": []
                    }
                ],
                "reverseDirection": false,
                "relationRatio": 1,
                "name": "Gear 1"
            }
        ```

    Attributes:
        relationType (RelationType): The type of mate relation.
        mates (list[MateRelationMate]): A list of mate relations.
        reverseDirection (bool): Indicates if the direction of the mate relation is reversed.
        relationRatio (Union[float, None]): The ratio of the GEAR mate relation. Defaults to None.
        relationLength (Union[float, None]): The length of the RACK_AND_PINION mate relation. Defaults to None.
        name (str): The name of the mate relation feature.

    Custom Attributes:
        id (str): The unique identifier of the feature.

    Examples:
        >>> MateRelationFeatureData(
        ...     relationType=RelationType.GEAR,
        ...     mates=[...],
        ...     reverseDirection=False,
        ...     relationRatio=1,
        ...     name="Gear 1",
        ... )
        MateRelationFeatureData(
            relationType=RelationType.GEAR,
            mates=[...],
            reverseDirection=False,
            relationRatio=1,
            name="Gear 1"
        )
    """

    relationType: RelationType = Field(..., description="The type of mate relation.")
    mates: list[MateRelationMate] = Field(..., description="A list of mate relations.")
    reverseDirection: bool = Field(..., description="Indicates if the direction of the mate relation is reversed.")
    relationRatio: Union[float, None] = Field(
        None, description="The ratio of the GEAR mate relation. Defaults to None."
    )
    relationLength: Union[float, None] = Field(
        None, description="The length of the RACK_AND_PINION mate relation. Defaults to None."
    )
    name: str = Field(..., description="The name of the mate relation feature.")

    id: str = Field(..., description="The unique identifier of the feature.")


class MateFeatureData(BaseModel):
    """
    Represents data for a mate feature within an assembly.

    JSON:
        ```json
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
        ```

    Attributes:
        matedEntities (list[MatedEntity]): A list of mated entities.
        mateType (MateType): The type of mate.
        name (str): The name of the mate feature.

    Custom Attributes:
        id (str): The unique identifier of the feature.
        limits (dict[str, float] | None): Joint limits for the mate {'min': lower_limit, 'max': upper_limit}.
            Fetched from Onshape features API if available.

    Examples:
        >>> MateFeatureData(
        ...     matedEntities=[...],
        ...     mateType=MateType.FASTENED,
        ...     name="Fastened 1",
        ... )
        MateFeatureData(
            matedEntities=[...],
            mateType=MateType.FASTENED,
            name="Fastened 1"
        )

    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    matedEntities: list[MatedEntity] = Field(..., description="A list of mated entities.")
    mateType: MateType = Field(..., description="The type of mate.")
    name: str = Field(..., description="The name of the mate feature.")

    id: str = Field(..., description="The unique identifier of the feature.")
    limits: Union[dict[str, float], None] = Field(
        None, description="Joint limits for the mate {'min': lower_limit, 'max': upper_limit}."
    )


class AssemblyFeature(BaseModel):
    """
    Represents a feature within an assembly, such as a mate or pattern.

    JSON:
        ```json
            {
            "id": "Mw+URe/Uaxx5gIdlu",
            "suppressed": false,
            "featureType": "mate",
            "featureData": {
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
        ```

    Attributes:
        id (str): The unique identifier of the feature.
        suppressed (bool): Indicates if the feature is suppressed.
        featureType (AssemblyFeatureType): The type of the feature.
        featureData (Union[MateGroupFeatureData, MateConnectorFeatureData, MateRelationFeatureData, MateFeatureData]):
            Data associated with the assembly feature.

    Examples:
        >>> AssemblyFeature(
        ...     id="Mw+URe/Uaxx5gIdlu",
        ...     suppressed=False,
        ...     featureType=AssemblyFeatureType.MATE,
        ...     featureData=MateFeatureData(
        ...         matedEntities=[...],
        ...         mateType=MateType.FASTENED,
        ...         name="Fastened 1",
        ...     ),
        ... )
        AssemblyFeature(
            id="Mw+URe/Uaxx5gIdlu",
            suppressed=False,
            featureType=AssemblyFeatureType.MATE,
            featureData=MateFeatureData(
                matedEntities=[...],
                mateType=MateType.FASTENED,
                name="Fastened 1"
            )
        )
    """

    id: str = Field(..., description="The unique identifier of the feature.")
    suppressed: bool = Field(..., description="Indicates if the feature is suppressed.")
    featureType: AssemblyFeatureType = Field(..., description="The type of the feature.")
    featureData: Union[MateGroupFeatureData, MateConnectorFeatureData, MateRelationFeatureData, MateFeatureData] = (
        Field(..., description="Data associated with the assembly feature.")
    )

    @model_validator(mode="before")
    def validate_feature_data(cls, values: Any) -> Any:
        """Custom validator to properly parse featureData based on featureType."""
        if isinstance(values, dict):
            feature_type = values.get("featureType")
            feature_data = values.get("featureData")

            if feature_data and isinstance(feature_data, dict):
                # Add the required 'id' field to featureData from the parent feature
                feature_data["id"] = values.get("id", "")

                # Parse based on feature type
                if feature_type == AssemblyFeatureType.MATE:
                    values["featureData"] = MateFeatureData(**feature_data)
                elif feature_type == AssemblyFeatureType.MATEGROUP:
                    values["featureData"] = MateGroupFeatureData(**feature_data)
                elif feature_type == AssemblyFeatureType.MATECONNECTOR:
                    values["featureData"] = MateConnectorFeatureData(**feature_data)
                elif feature_type == AssemblyFeatureType.MATERELATION:
                    values["featureData"] = MateRelationFeatureData(**feature_data)

        return values


class PatternType(str, Enum):
    CIRCULAR = "CIRCULAR"
    LINEAR = "LINEAR"
    MIRROR = "MIRROR"


class Pattern(BaseModel):
    """
    Represents a pattern feature within an assembly, defining repeated instances of parts or sub-assemblies.

    JSON:
        ```json
            {
                "seedToPatternInstances" : {
                    "MZAZxYCFuiiBSyUyF" : [ "Iska+TckyjXBC+1xA", "I1QmePT4s7IvbpMFC" ]
                },
                "name" : "Circular pattern 1",
                "suppressed" : false,
                "id" : "MVJMTPg/7tNhLqnTy",
                "type" : "CIRCULAR"
            }
        ```
    Attributes:
        seedToPatternInstances (dict[str, list[str]]): A mapping of seed instance IDs to their
            corresponding pattern instance IDs.
        name (str): The name of the pattern feature.
        suppressed (bool): Indicates if the pattern feature is suppressed.
        id (str): The unique identifier of the pattern feature.
        type (PatternType): The type of the pattern (e.g., CIRCULAR, LINEAR).
    """

    seedToPatternInstances: dict[Union[tuple[str, ...], str], Union[list[list[str]], list[str]]] = Field(
        ..., description="A mapping of seed instance IDs to their corresponding pattern instance IDs."
    )
    name: str = Field(..., description="The name of the pattern feature.")
    suppressed: bool = Field(..., description="Indicates if the pattern feature is suppressed.")
    id: str = Field(..., description="The unique identifier of the pattern feature.")
    type: PatternType = Field(..., description="The type of the pattern (e.g., CIRCULAR, LINEAR).")


class SubAssembly(IDBase):
    """
    Represents a sub-assembly within a root assembly.

    JSON:
        ```json
            {
                "instances": [],
                "patterns": [],
                "features": [],
                "fullConfiguration": "default",
                "configuration": "default",
                "documentId": "a1c1addf75444f54b504f25c",
                "elementId": "0b0c209535554345432581fe",
                "documentMicroversion": "349f6413cafefe8fb4ab3b07"
            }
        ```

    Attributes:
        instances (list[Union[PartInstance, AssemblyInstance]]):
            A list of part and assembly instances in the sub-assembly.
        patterns (list[Pattern]): A list of patterns in the sub-assembly.
        features (list[AssemblyFeature]): A list of features in the sub-assembly
        fullConfiguration (str): The full configuration of the sub-assembly.
        configuration (str): The configuration of the sub-assembly.
        documentId (str): The unique identifier of the document containing the sub-assembly.
        elementId (str): The unique identifier of the element containing the sub-assembly.
        documentMicroversion (str): The microversion of the document containing the sub-assembly.

    Properties:
        uid (str): A unique identifier for the sub-assembly based on documentId, documentMicroversion, elementId, and
            fullConfiguration.

    Examples:
        >>> SubAssembly(
        ...     instances=[...],
        ...     patterns=[...],
        ...     features=[...],
        ...     fullConfiguration="default",
        ...     configuration="default",
        ...     documentId="a1c1addf75444f54b504f25c",
        ...     elementId="0b0c209535554345432581fe",
        ...     documentMicroversion="349f6413cafefe8fb4ab3b07",
        ... )
        SubAssembly(
            instances=[...],
            patterns=[...],
            features=[...],
            fullConfiguration="default",
            configuration="default",
            documentId="a1c1addf75444f54b504f25c",
            elementId="0b0c209535554345432581fe",
            documentMicroversion="349f6413cafefe8fb4ab3b07"
        )

    """

    instances: list[Union[PartInstance, AssemblyInstance]] = Field(
        ..., description="A list of part and assembly instances in the sub-assembly."
    )
    patterns: list[Pattern] = Field(..., description="A list of patterns in the sub-assembly.")
    features: list[AssemblyFeature] = Field(..., description="A list of features in the sub-assembly")

    MassProperty: Union[MassProperties, None] = Field(
        None, description="The mass properties of the sub-assembly, this is retrieved via a separate API call."
    )

    isRigid: bool = Field(False, description="Indicates if the sub-assembly is a rigid assembly")

    RootOccurrences: Union[dict[Any, Occurrence], None] = Field(
        None, description="A list of occurrences from the subassembly frame, this is retrieved via a separate API call"
    )

    @property
    def uid(self) -> str:
        """
        Generates a unique identifier for the sub-assembly with documentId, documentMicroversion, elementId, and
        fullConfiguration.

        Returns:
            str: The unique identifier for the sub-assembly.
        """
        return generate_uid([self.documentId, self.documentMicroversion, self.elementId, self.fullConfiguration])


class RootAssembly(SubAssembly):
    """
    Represents the root assembly, which is the top-level assembly containing all parts and sub-assemblies.

    JSON:
        ```json
            {
                "instances": [],
                "patterns": [],
                "features": [],
                "occurrences": [],
                "fullConfiguration": "default",
                "configuration": "default",
                "documentId": "a1c1addf75444f54b504f25c",
                "elementId": "0b0c209535554345432581fe",
                "documentMicroversion": "349f6413cafefe8fb4ab3b07"
            }
        ```

    Attributes:
        instances (list[Union[PartInstance, AssemblyInstance]]):
            A list of part and assembly instances in the root assembly.
        patterns (list[Pattern]): A list of patterns in the root assembly.
        features (list[AssemblyFeature]): A list of features in the root assembly.
        occurrences (list[Occurrence]): A list of occurrences in the root assembly.
        fullConfiguration (str): The full configuration of the root assembly.
        configuration (str): The configuration of the root assembly.
        documentId (str): The unique identifier of the document containing the root assembly.
        elementId (str): The unique identifier of the element containing the root assembly.
        documentMicroversion (str): The microversion of the document containing the root assembly.

    Examples:
        >>> RootAssembly(
        ...     instances=[...],
        ...     patterns=[...],
        ...     features=[...],
        ...     occurrences=[...],
        ...     fullConfiguration="default",
        ...     configuration="default",
        ...     documentId="a1c1addf75444f54b504f25c",
        ...     elementId="0b0c209535554345432581fe",
        ...     documentMicroversion="349f6413cafefe8fb4ab3b07",
        ... )
        RootAssembly(
            instances=[...],
            patterns=[...],
            features=[...],
            occurrences=[...],
            fullConfiguration="default",
            configuration="default",
            documentId="a1c1addf75444f54b504f25c",
            elementId="0b0c209535554345432581fe",
            documentMicroversion="349f6413cafefe8fb4ab3b07"
        )
    """

    occurrences: list[Occurrence] = Field(..., description="A list of occurrences in the root assembly.")

    documentMetaData: Union[DocumentMetaData, None] = Field(
        None, description="The document associated with the assembly."
    )


class Assembly(BaseModel):
    """
    Represents the overall assembly, including all parts, sub-assemblies, and features.

    JSON:
        ```json
            {
                "rootAssembly": {
                    "instances": [],
                    "patterns": [],
                    "features": [],
                    "occurrences": [],
                    "fullConfiguration": "default",
                    "configuration": "default",
                    "documentId": "a1c1addf75444f54b504f25c",
                    "elementId": "0b0c209535554345432581fe",
                    "documentMicroversion": "349f6413cafefe8fb4ab3b07"
                },
                "subAssemblies": [],
                "parts": [],
                "partStudioFeatures": []
            }
        ```

    Attributes:
        rootAssembly (RootAssembly): The root assembly in the document.
        subAssemblies (list[SubAssembly]): A list of sub-assemblies in the document.
        parts (list[Part]): A list of parts in the document.
        partStudioFeatures (list[dict]): A list of part studio features in the document.

    Custom Attributes:
        document (Union[Document, None]): The document object associated with the assembly. Defaults to None.
        name (Union[str, None]): The name of the assembly. Defaults to None.


    Examples:
        >>> Assembly(
        ...     rootAssembly=RootAssembly(
        ...         instances=[...],
        ...         patterns=[...],
        ...         features=[...],
        ...         occurrences=[...],
        ...         fullConfiguration="default",
        ...         configuration="default",
        ...         documentId="a1c1addf75444f54b504f25c",
        ...         elementId="0b0c209535554345432581fe",
        ...         documentMicroversion="349f6413cafefe8fb4ab3b07",
        ...     ),
        ...     subAssemblies=[...],
        ...     parts=[...],
        ...     partStudioFeatures=[...],
        ... )
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
        )
    """

    rootAssembly: RootAssembly = Field(..., description="The root assembly in the document.")
    subAssemblies: list[SubAssembly] = Field(..., description="A list of sub-assemblies in the document.")
    parts: list[Part] = Field(..., description="A list of parts in the document.")
    partStudioFeatures: list[dict] = Field(..., description="A list of part studio features in the document.")

    document: Union[Document, None] = Field(None, description="The document associated with the assembly.")
    name: Union[str, None] = Field(None, description="The name of the assembly.")


class FeatureType(str, Enum):
    MATE = "mate"
    MATE_RELATION = "mateRelation"
    RELATION = "replicate"
    EMPTY = ""


class InferenceType(str, Enum):
    CENTROID = "CENTROID"


class ParameterMessage(BaseModel):
    parameterId: str
    hasUserCode: bool
    nodeId: str


Parameter = Any


class MateConnectorMessage(BaseModel):
    isHidden: bool
    implicit: bool
    isAuxiliaryTreeMateConnector: bool
    version: int
    featureType: str
    featureId: str
    name: str


class MateConnector(BaseModel):
    type: int
    typeName: str
    message: MateConnectorMessage


class SubFeature(BaseModel):
    pass


class SuppressionState(BaseModel):
    type: int


class FeatureMessage(BaseModel):
    version: int
    featureType: FeatureType
    featureId: str
    name: str
    parameters: list[Parameter]
    suppressed: bool
    namespace: str
    subFeatures: list[SubFeature]
    returnAfterSubfeatures: bool
    suppressionState: SuppressionState
    hasUserCode: bool
    nodeId: str
    mateConnectors: list[MateConnector] | None = None

    def parameter_dict(self) -> dict[str, dict]:
        return {param["message"]["parameterId"]: param for param in self.parameters}


class Feature(BaseModel):
    type: int
    typeName: str
    message: FeatureMessage


class FeatureStatus(str, Enum):
    OK = "OK"
    ERROR = "ERROR"


class FeatureStateMessage(BaseModel):
    featureStatus: FeatureStatus
    inactive: bool


class FeatureStateValue(BaseModel):
    type: Literal[1688]
    typeName: Literal["BTFeatureState"]
    message: FeatureStateMessage


class FeatureState(BaseModel):
    key: str
    value: FeatureStateValue


class Features(BaseModel):
    """
    Represents list of features within an assembly, such as a mate or relation. These are
    fetched separately from the main assembly call.
    """

    features: list[Feature] = Field(..., description="A list of features in the assembly.")
    featureStates: list[FeatureState] = Field(..., description="A list of feature states in the assembly.")
    isComplete: bool = Field(..., description="Indicates if the feature list is complete.")
    serializationVersion: str = Field(..., description="The serialization version of the feature list.")
    sourceMicroversion: str = Field(..., description="The source microversion of the feature list.")
    rejectMicroversionSkew: bool = Field(..., description="Indicates if microversion skew is rejected.")
    microversionSkew: bool = Field(..., description="Indicates if microversion skew is allowed.")
    libraryVersion: int = Field(..., description="The library version of the feature list.")


if __name__ == "__main__":
    mated_cs = MatedCS(
        xAxis=[1.0, 2.0, 3.0],
        yAxis=[4.0, 5.0, 6.0],
        zAxis=[7.0, 8.0, 9.0],
        origin=[10.0, 11.0, 12.0],
        tf=None,
    )

    print(mated_cs.xAxis, mated_cs.yAxis, mated_cs.zAxis, mated_cs.origin)
