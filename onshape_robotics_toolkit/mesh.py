"""
This module contains functions for transforming meshes and inertia matrices.

"""

from functools import partial

import numpy as np
from numpy.typing import NDArray
from stl.mesh import Mesh


def transform_vectors(
    vectors: NDArray[np.floating], rotation: NDArray[np.floating], translation: NDArray[np.floating]
) -> NDArray[np.floating]:
    """
    Apply a transformation matrix to a set of vectors.

    Args:
        vectors: Array of vectors to use for transformation
        rotation: Rotation matrix to apply to the vectors
        translation: Translation matrix to apply to the vectors

    Returns:
        Array of transformed vectors
    """

    return np.dot(vectors, rotation.T) + translation * len(vectors)


def transform_mesh(mesh: Mesh, transform: np.ndarray) -> Mesh:
    """
    Apply a transformation matrix to an STL mesh.

    Args:
        mesh: STL mesh to use for transformation
        transform: Transformation matrix to apply to the mesh

    Returns:
        Transformed STL mesh

    Examples:
        >>> mesh = Mesh.from_file("mesh.stl")
        >>> transform = np.eye(4)
        >>> transform_mesh(mesh, transform)
    """

    _transform_vectors = partial(
        transform_vectors, rotation=transform[:3, :3], translation=transform[0:3, 3:4].T.tolist()
    )

    mesh.v0 = _transform_vectors(mesh.v0)
    mesh.v1 = _transform_vectors(mesh.v1)
    mesh.v2 = _transform_vectors(mesh.v2)
    mesh.normals = _transform_vectors(mesh.normals)

    return mesh


def transform_inertia_matrix(
    inertia_matrix: NDArray[np.floating], rotation: NDArray[np.floating]
) -> NDArray[np.floating]:
    """
    Transform an inertia matrix

    Args:
        inertia_matrix: Inertia matrix to use for transformation
        rotation: Rotation matrix to apply to the inertia matrix

    Returns:
        Transformed inertia matrix
    """

    return rotation @ inertia_matrix @ rotation.T
