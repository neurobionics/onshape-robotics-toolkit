"""
This module defines data models for variables used in Onshape documents retrieved from Onshape REST API responses.

The data models are implemented as Pydantic BaseModel classes, which are used to

    1. Parse JSON responses from the API into Python objects.
    2. Validate the structure and types of the JSON responses.
    3. Provide type hints for better code clarity and autocompletion.

These models ensure that the data received from the API adheres to the expected format and types, facilitating easier
and safer manipulation of the data within the application.

Models:
    - **Variable**: Represents a variable used in Onshape's Variable Studio.

Enum:
    - **VARIABLE_TYPE**: Enumerates the possible variable types in Onshape (LENGTH, ANGLE, NUMBER, ANY).

"""

from enum import Enum
from typing import Union

from pydantic import BaseModel, Field, field_validator


class VARIABLE_TYPE(str, Enum):
    """
    Enumerates the possible variable types in Onshape

    Attributes:
        LENGTH (str): Length variable type
        ANGLE (str): Angle variable type
        NUMBER (str): Number variable type
        ANY (str): Any variable type

    Examples:
        >>> VARIABLE_TYPE.LENGTH
        'LENGTH'
        >>> VARIABLE_TYPE.ANGLE
        'ANGLE'
    """

    LENGTH = "LENGTH"
    ANGLE = "ANGLE"
    NUMBER = "NUMBER"
    ANY = "ANY"


class Variable(BaseModel):
    """
    Represents a variable used in Onshape's Variable Studio.

    JSON:
        ```json
            {
                "type": "ANGLE",
                "name": "forkAngle",
                "value": null,
                "description": "Fork angle for front wheel assembly in deg",
                "expression": "15 deg"
            }
        ```

    Attributes:
        type (str): The type of the variable (LENGTH, ANGLE, NUMBER, ANY).
        name (str): The name of the variable.
        value (str, optional): The value of the variable.
        description (str, optional): The description of the variable.
        expression (str, optional): The expression of the variable.

    Examples:
        >>> variable = Variable(
        ...     type="ANGLE",
        ...     name="forkAngle",
        ...     value=None,
        ...     description="Fork angle for front wheel assembly in deg",
        ...     expression="15 deg"
        ... )
        >>> variable
        Variable(
            type='ANGLE',
            name='forkAngle',
            value=None,
            description='Fork angle for front wheel assembly in deg',
            expression='15 deg'
        )
    """

    type: str = Field(..., description="The type of the variable (LENGTH, ANGLE, NUMBER, ANY)")
    name: str = Field(..., description="The name of the variable")
    value: Union[str, None] = Field(None, description="The value of the variable")
    description: Union[str, None] = Field(None, description="The description of the variable")
    expression: Union[str, None] = Field(None, description="The expression of the variable")

    @field_validator("name")
    def validate_name(cls, value: str) -> str:
        """
        Validate the variable name to ensure it is not empty.

        Args:
            value (str): The variable name to validate.

        Returns:
            str: The validated variable name.

        Raises:
            ValueError: If the variable name is empty.
        """
        if not value:
            raise ValueError("Variable name cannot be empty")

        return value

    @field_validator("type")
    def validate_type(cls, value: str) -> str:
        """
        Validate the variable type to ensure it is one of the valid types.

        Args:
            value (str): The variable type to validate.

        Returns:
            str: The validated variable type.

        Raises:
            ValueError: If the variable type is not one of the valid types.
        """
        if value not in VARIABLE_TYPE.__members__.values():
            raise ValueError(f"Invalid variable type: {value}")

        return value

    # @field_validator("expression")
    # def validate_expression(cls, value: str) -> str:
    #     """
    #     Validate the variable expression to ensure it is not empty and in the correct format.

    #     Args:
    #         value (str): The variable expression to validate.

    #     Returns:
    #         str: The validated variable expression.

    #     Raises:
    #         ValueError: If the variable expression is empty or not in the correct format.
    #     """
    #     if not value:
    #         raise ValueError("Variable expression cannot be empty")

    #     # ensure that there is a space between the value and the unit
    #     if not value.split(" ")[1]:
    #         raise ValueError("Invalid expression format, must be in the form of 'value unit' e.g. '15 deg'")

    #     return value
