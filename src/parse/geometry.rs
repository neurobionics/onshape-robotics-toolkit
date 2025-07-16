use nalgebra::{Matrix4, Vector3, Matrix3};
// use crate::parse::models::{MatedCS}; // Currently unused
use crate::model::MassProperties;

/// Geometric utilities for transformation matrix operations
pub struct GeometryUtils;

impl GeometryUtils {
    /// Create a 4x4 transformation matrix from rotation matrix and translation vector
    pub fn create_transform_matrix(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> Matrix4<f64> {
        let mut transform = Matrix4::identity();

        // Set rotation part
        transform.fixed_view_mut::<3, 3>(0, 0).copy_from(rotation);

        // Set translation part
        transform.fixed_view_mut::<3, 1>(0, 3).copy_from(translation);

        transform
    }

    /// Extract rotation matrix from 4x4 transformation matrix
    pub fn extract_rotation(transform: &Matrix4<f64>) -> Matrix3<f64> {
        transform.fixed_view::<3, 3>(0, 0).into()
    }

    /// Extract translation vector from 4x4 transformation matrix
    pub fn extract_translation(transform: &Matrix4<f64>) -> Vector3<f64> {
        transform.fixed_view::<3, 1>(0, 3).into()
    }

    /// Calculate the inverse of a transformation matrix
    pub fn inverse_transform(transform: &Matrix4<f64>) -> Result<Matrix4<f64>, String> {
        transform.try_inverse()
            .ok_or_else(|| "Matrix is not invertible".to_string())
    }

    /// Multiply two transformation matrices
    pub fn multiply_transforms(t1: &Matrix4<f64>, t2: &Matrix4<f64>) -> Matrix4<f64> {
        t1 * t2
    }

    /// Convert a transformation matrix to STL-to-link transformation
    /// This mirrors the Python logic: _stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
    pub fn stl_to_link_transform(link_to_stl_tf: &Matrix4<f64>) -> Result<Matrix4<f64>, String> {
        Self::inverse_transform(link_to_stl_tf)
    }

    /// Calculate center of mass with respect to a transformation matrix
    /// Mirrors: part.MassProperty.center_of_mass_wrt(_stl_to_link_tf)
    pub fn center_of_mass_wrt(
        center_of_mass: &[f64; 3],
        transform: &Matrix4<f64>
    ) -> Result<Vector3<f64>, String> {
        if center_of_mass.len() != 3 {
            return Err("Center of mass must have 3 components".to_string());
        }

        let com_homogeneous = Vector3::new(center_of_mass[0], center_of_mass[1], center_of_mass[2]);
        let transformed_com = transform.transform_vector(&com_homogeneous);

        Ok(transformed_com)
    }

    /// Calculate inertia tensor with respect to a rotation matrix
    /// Mirrors: part.MassProperty.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))
    pub fn inertia_wrt(
        inertia_flat: &[f64],
        rotation: &Matrix3<f64>
    ) -> Result<Matrix3<f64>, String> {
        if inertia_flat.len() != 9 {
            return Err("Inertia tensor must have 9 components".to_string());
        }

        // Reconstruct the inertia matrix from flat array
        // Assuming the flat array is in row-major order: [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz]
        let inertia_matrix = Matrix3::new(
            inertia_flat[0], inertia_flat[1], inertia_flat[2],
            inertia_flat[3], inertia_flat[4], inertia_flat[5],
            inertia_flat[6], inertia_flat[7], inertia_flat[8],
        );

        // Transform inertia: R * I * R^T
        let rotated_inertia = rotation * inertia_matrix * rotation.transpose();

        Ok(rotated_inertia)
    }

    /// Extract inertia components for URDF format
    /// Returns (ixx, ixy, ixz, iyy, iyz, izz)
    pub fn extract_inertia_components(inertia: &Matrix3<f64>) -> (f64, f64, f64, f64, f64, f64) {
        (
            inertia[(0, 0)], // ixx
            inertia[(0, 1)], // ixy
            inertia[(0, 2)], // ixz
            inertia[(1, 1)], // iyy
            inertia[(1, 2)], // iyz
            inertia[(2, 2)], // izz
        )
    }

    /// Create transformation matrix from origin (position + orientation)
    /// Mirrors the Origin.from_matrix logic in Python
    pub fn origin_from_matrix(matrix: &Matrix4<f64>) -> (Vector3<f64>, Vector3<f64>) {
        let translation = Self::extract_translation(matrix);

        // Extract Euler angles from rotation matrix (ZYX convention)
        let rotation = Self::extract_rotation(matrix);
        let (roll, pitch, yaw) = Self::rotation_matrix_to_euler_zyx(&rotation);

        (translation, Vector3::new(roll, pitch, yaw))
    }

    /// Convert rotation matrix to Euler angles (ZYX convention)
    /// This is commonly used in robotics for roll-pitch-yaw representation
    pub fn rotation_matrix_to_euler_zyx(rotation: &Matrix3<f64>) -> (f64, f64, f64) {
        let r11 = rotation[(0, 0)];
        let _r12 = rotation[(0, 1)];
        let r13 = rotation[(0, 2)];
        let r21 = rotation[(1, 0)];
        let r22 = rotation[(1, 1)];
        let r23 = rotation[(1, 2)];
        let r31 = rotation[(2, 0)];
        let r32 = rotation[(2, 1)];
        let r33 = rotation[(2, 2)];

        // ZYX Euler angles
        let sy = (r13 * r13 + r23 * r23).sqrt();

        let (roll, pitch, yaw) = if sy > 1e-6 {
            let x = r32.atan2(r33);
            let y = (-r31).atan2(sy);
            let z = r21.atan2(r11);
            (x, y, z)
        } else {
            let x = (-r23).atan2(r22);
            let y = (-r31).atan2(sy);
            let z = 0.0;
            (x, y, z)
        };

        (roll, pitch, yaw)
    }

    /// Create rotation matrix from Euler angles (ZYX convention)
    pub fn euler_zyx_to_rotation_matrix(roll: f64, pitch: f64, yaw: f64) -> Matrix3<f64> {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();

        Matrix3::new(
            cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr,
        )
    }

    /// Apply transformation to a point
    pub fn transform_point(transform: &Matrix4<f64>, point: &Vector3<f64>) -> Vector3<f64> {
        let homogeneous = transform * Vector3::new(point.x, point.y, point.z).to_homogeneous();
        Vector3::new(homogeneous.x, homogeneous.y, homogeneous.z)
    }
}

/// Helper for mass property calculations
pub struct MassPropertyUtils;

impl MassPropertyUtils {
    /// Extract mass from MassProperties (first element of mass array)
    pub fn extract_mass(mass_props: &MassProperties) -> f64 {
        mass_props.mass.get(0).copied().unwrap_or(0.0)
    }

    /// Extract center of mass from MassProperties
    pub fn extract_center_of_mass(mass_props: &MassProperties) -> [f64; 3] {
        [
            mass_props.centroid.get(0).copied().unwrap_or(0.0),
            mass_props.centroid.get(1).copied().unwrap_or(0.0),
            mass_props.centroid.get(2).copied().unwrap_or(0.0),
        ]
    }

    /// Extract inertia tensor from MassProperties
    /// The inertia array from Onshape contains 9 elements representing the 3x3 inertia tensor
    pub fn extract_inertia_tensor(mass_props: &MassProperties) -> Result<Matrix3<f64>, String> {
        if mass_props.inertia.len() < 9 {
            return Err("Inertia array must have at least 9 elements".to_string());
        }

        // Onshape inertia format: [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz]
        Ok(Matrix3::new(
            mass_props.inertia[0], mass_props.inertia[1], mass_props.inertia[2],
            mass_props.inertia[3], mass_props.inertia[4], mass_props.inertia[5],
            mass_props.inertia[6], mass_props.inertia[7], mass_props.inertia[8],
        ))
    }

    /// Calculate center of mass with respect to a transformation
    /// This mirrors the Python: part.MassProperty.center_of_mass_wrt(_stl_to_link_tf)
    pub fn center_of_mass_wrt(
        mass_props: &MassProperties,
        transform: &Matrix4<f64>
    ) -> Result<Vector3<f64>, String> {
        let com = Self::extract_center_of_mass(mass_props);
        GeometryUtils::center_of_mass_wrt(&com, transform)
    }

    /// Calculate inertia tensor with respect to a transformation
    /// This mirrors the Python: part.MassProperty.inertia_wrt(np.matrix(_stl_to_link_tf[:3, :3]))
    pub fn inertia_wrt(
        mass_props: &MassProperties,
        transform: &Matrix4<f64>
    ) -> Result<Matrix3<f64>, String> {
        let inertia_tensor = Self::extract_inertia_tensor(mass_props)?;
        let rotation = GeometryUtils::extract_rotation(transform);

        // Transform inertia: R * I * R^T
        Ok(rotation * inertia_tensor * rotation.transpose())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector3, Matrix3};

    #[test]
    fn test_create_transform_matrix() {
        let rotation = Matrix3::identity();
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let transform = GeometryUtils::create_transform_matrix(&rotation, &translation);

        assert_eq!(transform[(0, 3)], 1.0);
        assert_eq!(transform[(1, 3)], 2.0);
        assert_eq!(transform[(2, 3)], 3.0);
        assert_eq!(transform[(3, 3)], 1.0);
    }

    #[test]
    fn test_extract_components() {
        let mut transform = Matrix4::identity();
        transform[(0, 3)] = 5.0;
        transform[(1, 3)] = 6.0;
        transform[(2, 3)] = 7.0;

        let translation = GeometryUtils::extract_translation(&transform);
        assert_eq!(translation, Vector3::new(5.0, 6.0, 7.0));

        let rotation = GeometryUtils::extract_rotation(&transform);
        assert_eq!(rotation, Matrix3::identity());
    }

    #[test]
    fn test_euler_conversion() {
        let (roll, pitch, yaw) = (0.1, 0.2, 0.3);
        let rotation = GeometryUtils::euler_zyx_to_rotation_matrix(roll, pitch, yaw);
        let (r2, p2, y2) = GeometryUtils::rotation_matrix_to_euler_zyx(&rotation);

        assert!((roll - r2).abs() < 1e-10);
        assert!((pitch - p2).abs() < 1e-10);
        assert!((yaw - y2).abs() < 1e-10);
    }
}
