"""
Data model for Onshape's Element:
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
"""

from enum import Enum

from pydantic import BaseModel, field_validator

__all__ = ["ELEMENT_TYPE", "Element"]


class ELEMENT_TYPE(str, Enum):
    PARTSTUDIO = "PARTSTUDIO"
    ASSEMBLY = "ASSEMBLY"
    VARIABLESTUDIO = "VARIABLESTUDIO"
    DRAWING = "DRAWING"
    BILLOFMATERIALS = "BILLOFMATERIALS"
    APPLICATION = "APPLICATION"
    BLOB = "BLOB"


class Element(BaseModel):
    id: str
    name: str
    elementType: str
    microversionId: str

    @field_validator("elementType")
    def validate_type(cls, value: str) -> str:
        if value not in ELEMENT_TYPE.__members__.values():
            raise ValueError(f"Invalid element type: {value}")

        return value

    @field_validator("id")
    def validate_id(cls, value: str) -> str:
        if len(value) != 24:
            raise ValueError(f"Invalid element ID: {value}, must be 24 characters long")

        return value

    @field_validator("microversionId")
    def validate_mid(cls, value: str) -> str:
        if len(value) != 24:
            raise ValueError(f"Invalid microversion ID: {value}, must be 24 characters long")

        return value


if __name__ == "__main__":
    element_json = {
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
    element = Element(**element_json)
    print(element)
