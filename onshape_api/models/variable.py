"""
Data model for Onshape's Variable:
    {
        "type": "ANGLE",
        "name": "forkAngle",
        "value": null,
        "description": "Fork angle for front wheel assembly in deg",
        "expression": "15 deg"
    }
"""

from enum import Enum
from typing import Optional

from pydantic import BaseModel, field_validator


class VARIABLE_TYPE(str, Enum):
    LENGTH = "LENGTH"
    ANGLE = "ANGLE"
    NUMBER = "NUMBER"
    ANY = "ANY"


class Variable(BaseModel):
    """
    Variable model for Onshape's Variable Studio

    Example:
        {
            "type": "ANGLE",
            "name": "forkAngle",
            "value": null,
            "description": "Fork angle for front wheel assembly in deg",
            "expression": "15 deg"
        }
    """

    type: VARIABLE_TYPE
    name: str
    value: Optional[str] = None
    description: Optional[str] = None
    expression: Optional[str] = None

    @field_validator("name")
    def validate_name(cls, value: str) -> str:
        if not value:
            raise ValueError("Variable name cannot be empty")

        return value

    @field_validator("type")
    def validate_type(cls, value: VARIABLE_TYPE) -> VARIABLE_TYPE:
        if value not in VARIABLE_TYPE.__members__.values():
            raise ValueError(f"Invalid variable type: {value}")

        return value

    @field_validator("expression")
    def validate_expression(cls, value: str) -> str:
        if not value:
            raise ValueError("Variable expression cannot be empty")

        # ensure that there is a space between the value and the unit
        if not value.split(" ")[1]:
            raise ValueError("Invalid expression format, must be in the form of 'value unit' e.g. '15 deg'")

        return value


if __name__ == "__main__":
    variable_json = {
        "type": "ANGLE",
        "name": "forkAngle",
        "value": None,
        "description": "Fork angle for front wheel assembly in deg",
        "expression": "15 deg",
    }
    variable = Variable(**variable_json)
    print(variable.model_dump())
