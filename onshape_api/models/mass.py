import numpy as np
from pydantic import BaseModel, field_validator


class PrincipalAxis(BaseModel):
    """
    {
        "x" : 5.481818620570986E-9,
        "y" : -0.9999999999999999,
        "z" : 8.066832175421143E-10
    }
    """

    x: float
    y: float
    z: float

    @property
    def values(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class MassModel(BaseModel):
    """
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
    """

    volume: list[float]
    mass: list[float]
    centroid: list[float]
    inertia: list[float]
    principalInertia: list[float]
    principalAxes: list[PrincipalAxis]

    @field_validator("principalAxes")
    def check_principal_axes(cls, v: list[PrincipalAxis]) -> list[PrincipalAxis]:
        if len(v) != 3:
            raise ValueError("Principal axes must have 3 elements")
        return v

    @property
    def principal_inertia(self) -> np.ndarray:
        return np.array(self.principalInertia)

    @property
    def center_of_mass(self) -> tuple[float, float, float]:
        return (self.centroid[0], self.centroid[1], self.centroid[2])

    @property
    def inertia_matrix(self) -> np.matrix:
        return np.matrix(np.array(self.inertia[:9]).reshape(3, 3))

    @property
    def principal_axes(self) -> np.matrix:
        return np.matrix(np.array([axis.values for axis in self.principalAxes]))

    def principal_axes_wrt(self, reference: np.matrix) -> np.matrix:
        if reference.shape != (3, 3):
            raise ValueError("Reference frame must be a 3x3 matrix")

        return reference @ self.principal_axes

    def inertia_wrt(self, reference: np.matrix) -> np.matrix:
        if reference.shape != (3, 3):
            raise ValueError("Reference frame must be a 3x3 matrix")

        return reference @ self.inertia_matrix @ reference.T

    def center_of_mass_wrt(self, reference: np.matrix) -> np.ndarray:
        if reference.shape != (4, 4):
            raise ValueError("Reference frame must be a 4x4 matrix")

        com = np.matrix([*list(self.center_of_mass), 1.0])
        com_wrt = (reference * com.T)[:3]
        return np.array([com_wrt[0, 0], com_wrt[1, 0], com_wrt[2, 0]])


if __name__ == "__main__":
    mass = MassModel(
        volume=[0.003411385108378978, 0.003410724395374695, 0.0034120458213832646],
        mass=[9.585992154544929, 9.584199206938452, 9.587785102151415],
        centroid=[-2.07609188073475e-9, 0.0, 0.0],
        inertia=[0.0994460593470721, 0.09944605954654827, 0.19238058837442526],
        principalInertia=[0.09944605933465941, 0.09944605954654827, 0.19238058837442526],
        principalAxes=[
            PrincipalAxis(x=-0.9702683926946019, y=-5.514078101148926e-9, z=-0.2420314982349062),
            PrincipalAxis(x=-0.2420314982349062, y=-5.44073563520025e-10, z=0.9702683926946021),
            PrincipalAxis(x=5.481818620570986e-9, y=-0.9999999999999999, z=8.066832175421143e-10),
        ],
    )
    print(mass.center_of_mass_wrt(np.eye(4)), mass.center_of_mass)
