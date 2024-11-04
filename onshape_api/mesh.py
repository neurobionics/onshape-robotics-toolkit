from functools import partial

import numpy as np
from stl import mesh


def transform_vectors(vectors: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    Apply a transformation matrix to a set of vectors.

    Args:
        - vectors (np.ndarray): Array of vectors to transform
        - rotation (np.ndarray): Rotation matrix
        - translation (np.ndarray): Translation vector

    Returns:
        - np.ndarray: Transformed vectors
    """
    return np.dot(vectors, rotation.T) + translation * len(vectors)


def transform_mesh(mesh: mesh.Mesh, transform: np.ndarray) -> mesh.Mesh:
    """
    Apply a transformation matrix to a mesh.

    Args:
        - mesh (stl.mesh.Mesh): Mesh to transform
        - transform (np.ndarray): Transformation matrix

    Returns:
        - stl.mesh.Mesh: Transformed mesh
    """
    _transform_vectors = partial(
        transform_vectors, rotation=transform[:3, :3], translation=transform[0:3, 3:4].T.tolist()
    )

    mesh.v0 = _transform_vectors(mesh.v0)
    mesh.v1 = _transform_vectors(mesh.v1)
    mesh.v2 = _transform_vectors(mesh.v2)
    mesh.normals = _transform_vectors(mesh.normals)

    return mesh


def transform_inertia_matrix(inertia_matrix: np.matrix, rotation: np.matrix) -> np.matrix:
    """
    Transform an inertia matrix to a new reference frame.

    Args:
        - inertia_matrix (np.matrix): Inertia matrix to transform
        - rotation (np.matrix): Rotation matrix

    Returns:
        - np.matrix: Transformed inertia matrix
    """
    return np.matrix(rotation @ inertia_matrix @ rotation.T)
