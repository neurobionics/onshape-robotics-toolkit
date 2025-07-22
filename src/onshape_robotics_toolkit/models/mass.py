"""
This module defines data model for mass properties retrieved from Onshape REST API responses.

The data models are implemented as Pydantic BaseModel classes, which are used to

    1. Parse JSON responses from the API into Python objects.
    2. Validate the structure and types of the JSON responses.
    3. Provide type hints for better code clarity and autocompletion.

These models ensure that the data received from the API adheres to the expected format and types, facilitating easier
and safer manipulation of the data within the application.

Models:
    - **PrincipalAxis**: Represents the principal axis of a part or an entity.
    - **MassProperties**: Represents the mass properties of a part or an entity.

"""

import numpy as np
from pydantic import BaseModel, Field, field_validator


class PrincipalAxis(BaseModel):
    """
    Represents the principal axis of a part or an entity.

    JSON:
        ```json
            {
                "x" : 5.481818620570986E-9,
                "y" : -0.9999999999999999,
                "z" : 8.066832175421143E-10
            }
        ```

    Attributes:
        x (float): The x-component of the principal axis.
        y (float): The y-component of the principal axis.
        z (float): The z-component of the principal axis.

    Examples:
        >>> axis = PrincipalAxis(x=0.0, y=0.0, z=1.0)
        >>> axis.values
        array([0., 0., 1.])

    """

    x: float = Field(..., description="The x-component of the principal axis.")
    y: float = Field(..., description="The y-component of the principal axis.")
    z: float = Field(..., description="The z-component of the principal axis.")

    @property
    def values(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class MassProperties(BaseModel):
    """
    Represents the mass properties of a part or an entity.

    JSON:
        ```json
            {
                "volume" : [ 0.003411385108378978, 0.003410724395374695, 0.0034120458213832646 ],
                "periphery" : [ 0.3551154530453107, 0.35506836484837767, 0.3551625412422433 ],
                "centroid" : [ -2.07609188073475E-9, ... ],
                "inertia" : [ 0.0994460593470721, ... ],
                "mass" : [ 9.585992154544929, 9.584199206938452, 9.587785102151415 ],
                "hasMass" : true,
                "massMissingCount" : 0,
                "principalInertia" : [ 0.09944605933465941, 0.09944605954654827, 0.19238058837442526 ],
                "principalAxes" : [
                    {
                    "x" : -0.9702683926946019,
                    "y" : -5.514078101148926E-9,
                    "z" : -0.2420314982349062
                    }, {
                    "x" : -0.2420314982349062,
                    "y" : -5.44073563520025E-10,
                    "z" : 0.9702683926946021
                    }, {
                    "x" : 5.481818620570986E-9,
                    "y" : -0.9999999999999999,
                    "z" : 8.066832175421143E-10
                    }
                ]
            }
        ```

    Attributes:
        volume (list[float]): The volume of the part.
        mass (list[float]): The mass of the part
        centroid (list[float]): The centroid of the part.
        inertia (list[float]): The inertia of the part.
        principalInertia (list[float, float, float]): The principal inertia of the part.
        principalAxes (list[PrincipalAxis]): The principal axes of the part.

    Properties:
        principal_inertia: The principal inertia as a numpy array.
        center_of_mass: The center of mass as a tuple of three floats.
        inertia_matrix: The inertia matrix as a 3x3 numpy matrix.
        principal_axes: The principal axes as a 3x3 numpy matrix.

    Methods:
        principal_axes_wrt: Returns the principal axes with respect to a given reference frame.
        inertia_wrt: Returns the inertia matrix with respect to a given reference frame.
        center_of_mass_wrt: Returns the center of mass with respect to a given reference frame.

    Examples:
        >>> mass_properties = MassProperties(
        ...     volume=[0.003411385108378978, 0.003410724395374695, 0.0034120458213832646],
        ...     mass=[9.585992154544929, 9.584199206938452, 9.587785102151415],
        ...     centroid=[...],
        ...     inertia=[...],
        ...     principalInertia=[0.09944605933465941, 0.09944605954654827, 0.19238058837442526],
        ...     principalAxes=[...]
        ... )
        >>> mass_properties.principal_inertia
        array([0.09944606, 0.09944606, 0.19238059])

        >>> mass_properties.center_of_mass_wrt(np.eye(4))
        array([0., 0., 0.])

        >>> mass_properties.principal_axes_wrt(np.eye(3))
        array([0.09944605933465941, 0.09944605954654827, 0.19238058837442526])
    """

    volume: list[float] = Field(..., description="The volume of the part.")
    mass: list[float] = Field(..., description="The mass of the part.")
    centroid: list[float] = Field(..., description="The centroid of the part.")
    inertia: list[float] = Field(..., description="The inertia of the part.")
    principalInertia: list[float, float, float] = Field(..., description="The principal inertia of the part.")
    principalAxes: list[PrincipalAxis] = Field(..., description="The principal axes of the part.")

    @field_validator("principalAxes")
    def check_principal_axes(cls, v: list[PrincipalAxis]) -> list[PrincipalAxis]:
        """
        Validate the principal axes to ensure they have 3 elements.

        Args:
            v: The principal axes to validate.

        Returns:
            The validated principal axes.

        Raises:
            ValueError: If the principal axes do not have 3 elements.
        """
        if len(v) != 3:
            raise ValueError("Principal axes must have 3 elements")
        return v

    @property
    def principal_inertia(self) -> np.ndarray:
        """
        Returns the principal inertia as a numpy array.

        Returns:
            The principal inertia.
        """
        return np.array(self.principalInertia)

    @property
    def center_of_mass(self) -> tuple[float, float, float]:
        """
        Returns the center of mass as a tuple of three floats.

        Returns:
            The center of mass.
        """
        return (self.centroid[0], self.centroid[1], self.centroid[2])

    @property
    def inertia_matrix(self) -> np.matrix:
        """
        Returns the inertia matrix as a 3x3 numpy matrix.

        Returns:
            The inertia matrix.
        """
        return np.matrix(np.array(self.inertia[:9]).reshape(3, 3))

    @property
    def principal_axes(self) -> np.matrix:
        """
        Returns the principal axes as a 3x3 numpy matrix.

        Returns:
            The principal axes.
        """
        return np.matrix(np.array([axis.values for axis in self.principalAxes]))

    def principal_axes_wrt(self, reference: np.matrix) -> np.matrix:
        """
        Returns the principal axes with respect to a given reference frame.

        Args:
            reference: The reference frame as a 3x3 matrix.

        Returns:
            The principal axes with respect to the reference frame.

        Raises:
            ValueError: If the reference frame is not a 3x3 matrix.

        Examples:
            >>> mass_properties.principal_axes_wrt(np.eye(3))
            array([0.09944605933465941, 0.09944605954654827, 0.19238058837442526])
        """
        if reference.shape != (3, 3):
            raise ValueError("Reference frame must be a 3x3 matrix")

        return reference @ self.principal_axes

    def inertia_wrt(self, reference: np.matrix) -> np.matrix:
        """
        Returns the inertia matrix with respect to a given reference frame.

        Args:
            reference: The reference frame as a 3x3 matrix.

        Returns:
            The inertia matrix with respect to the reference frame.

        Raises:
            ValueError: If the reference frame is not a 3x3 matrix.

        Examples:
            >>> mass_properties.inertia_wrt(np.eye(3))
            array([0.09944605933465941, 0.09944605954654827, 0.19238058837442526])
        """
        if reference.shape != (3, 3):
            raise ValueError("Reference frame must be a 3x3 matrix")

        return reference @ self.inertia_matrix @ reference.T

    def center_of_mass_wrt(self, reference: np.matrix) -> np.ndarray:
        """
        Returns the center of mass with respect to a given reference frame.

        Args:
            reference: The reference frame as a 4x4 matrix.

        Returns:
            The center of mass with respect to the reference frame.

        Raises:
            ValueError: If the reference frame is not a 4x4 matrix.

        Examples:
            >>> mass_properties.center_of_mass_wrt(np.eye(4))
            array([0., 0., 0.])
        """
        if reference.shape != (4, 4):
            raise ValueError("Reference frame must be a 4x4 matrix")

        com = np.matrix([*list(self.center_of_mass), 1.0])
        com_wrt = (reference * com.T)[:3]
        return np.array([com_wrt[0, 0], com_wrt[1, 0], com_wrt[2, 0]])
