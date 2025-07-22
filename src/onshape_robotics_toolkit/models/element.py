"""
This module defines data model for elements retrieved from Onshape REST API responses.

The data models are implemented as Pydantic BaseModel classes, which are used to

    1. Parse JSON responses from the API into Python objects.
    2. Validate the structure and types of the JSON responses.
    3. Provide type hints for better code clarity and autocompletion.

These models ensure that the data received from the API adheres to the expected format and types, facilitating easier
and safer manipulation of the data within the application.

Models:
    - **Element**: Represents an Onshape element, containing the element ID, name, type, and microversion ID.

Enum:
    - **ElementType**: Enumerates the possible element types in Onshape (PARTSTUDIO, ASSEMBLY, DRAWING, etc.).
"""

from enum import Enum

from pydantic import BaseModel, Field, field_validator

__all__ = ["Element", "ElementType"]


class ElementType(str, Enum):
    """
    Enumerates the possible element types in Onshape

    Attributes:
        PARTSTUDIO (str): Part Studio
        ASSEMBLY (str): Assembly
        VARIABLESTUDIO (str): Variable Studio
        DRAWING (str): Drawing
        BILLOFMATERIALS (str): Bill of Materials
        APPLICATION (str): Application
        BLOB (str): Blob
        FEATURESTUDIO (str): Feature Studio

    Examples:
        >>> ElementType.PARTSTUDIO
        'PARTSTUDIO'
        >>> ElementType.ASSEMBLY
        'ASSEMBLY'
    """

    PARTSTUDIO = "PARTSTUDIO"
    ASSEMBLY = "ASSEMBLY"
    VARIABLESTUDIO = "VARIABLESTUDIO"
    DRAWING = "DRAWING"
    BILLOFMATERIALS = "BILLOFMATERIALS"
    APPLICATION = "APPLICATION"
    BLOB = "BLOB"
    FEATURESTUDIO = "FEATURESTUDIO"


class Element(BaseModel):
    """
    Represents an Onshape element, containing the element ID, name, type, and microversion ID.

    JSON:
        ```json
            {
                "name": "wheelAndFork",
                "id": "0b0c209535554345432581fe",
                "type": "Part Studio",
                "elementType": "PARTSTUDIO",
                "dataType": "onshape/partstudio",
                "microversionId": "9b3be6165c7a2b1f6dd61305",
                "lengthUnits": "millimeter",
                "angleUnits": "degree",
                "massUnits": "kilogram",
                "timeUnits": "second",
                "forceUnits": "newton",
                "pressureUnits": "pascal",
                "momentUnits": "newtonMeter",
                "accelerationUnits": "meterPerSecondSquared",
                "angularVelocityUnits": "degreePerSecond",
                "energyUnits": "footPoundForce",
                "areaUnits": "squareMillimeter",
                "volumeUnits": "cubicMillimeter",
            }
        ```

    Attributes:
        id (str): The unique identifier of the element.
        name (str): The name of the element.
        elementType (str): The type of the element (e.g., PARTSTUDIO, ASSEMBLY, DRAWING).
        microversionId (str): The unique identifier of the microversion of the element.

    Examples:
        >>> element = Element(id="0b0c209535554345432581fe", name="wheelAndFork", elementType="PARTSTUDIO",
        ...                   microversionId="9b3be6165c7a2b1f6dd61305")
        >>> element
        Element(id='0b0c209535554345432581fe', name='wheelAndFork', elementType='PARTSTUDIO',
                microversionId='9b3be6165c7a2b1f6dd61305')
    """

    id: str = Field(..., description="The unique identifier of the element")
    name: str = Field(..., description="The name of the element")
    elementType: str = Field(..., description="The type of the element")
    microversionId: str = Field(..., description="The unique identifier of the microversion of the element")

    @field_validator("elementType")
    def validate_type(cls, value: str) -> str:
        """
        Validate the element type.

        Args:
            value: The element type to validate.

        Returns:
            The validated element type.

        Raises:
            ValueError: If the element type is not one of the valid types.
        """

        if value not in ElementType.__members__.values():
            raise ValueError(f"Invalid element type: {value}")

        return value

    @field_validator("id")
    def validate_id(cls, value: str) -> str:
        """
        Validate the element ID.

        Args:
            value: The element ID to validate.

        Returns:
            The validated element ID.

        Raises:
            ValueError: If the element ID is not 24 characters long.
        """

        if len(value) != 24:
            raise ValueError(f"Invalid element ID: {value}, must be 24 characters long")

        return value

    @field_validator("microversionId")
    def validate_mid(cls, value: str) -> str:
        """
        Validate the microversion ID.

        Args:
            value: The microversion ID to validate.

        Returns:
            The validated microversion ID.

        Raises:
            ValueError: If the microversion ID is not 24 characters long.
        """

        if len(value) != 24:
            raise ValueError(f"Invalid microversion ID: {value}, must be 24 characters long")

        return value
