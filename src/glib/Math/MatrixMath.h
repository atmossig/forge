#pragma once

#include "AlgebraMath.h"

namespace glib {
namespace math {

//------------------------------------------------------------------------------
// Matrix Transformation Utilities
//------------------------------------------------------------------------------

// Creates a billboard matrix (sprite always facing camera)
template <typename T>
Matrix4x4<T> CreateBillboardMatrix(
	const Vector3<T>& objectPosition,
	const Vector3<T>& cameraPosition,
	const Vector3<T>& cameraUp,
	const Vector3<T>* cameraForward = nullptr);

// Creates a constrained billboard matrix (sprite keeps upright)
template <typename T>
Matrix4x4<T> CreateConstrainedBillboardMatrix(
	const Vector3<T>& objectPosition,
	const Vector3<T>& cameraPosition,
	const Vector3<T>& rotateAxis,
	const Vector3<T>* cameraForward = nullptr,
	const Vector3<T>* objectForward = nullptr);

// Creates a view matrix (camera transformation)
template <typename T>
Matrix4x4<T> CreateLookAtMatrix(
	const Vector3<T>& cameraPosition,
	const Vector3<T>& cameraTarget,
	const Vector3<T>& cameraUpVector);

// Creates a perspective projection matrix
template <typename T>
Matrix4x4<T> CreatePerspectiveMatrix(
	T fieldOfView,
	T aspectRatio,
	T nearPlaneDistance,
	T farPlaneDistance);

// Creates a perspective projection matrix with off-center parameters
template <typename T>
Matrix4x4<T> CreatePerspectiveOffCenterMatrix(
	T left, T right, 
	T bottom, T top,
	T nearPlaneDistance, T farPlaneDistance);

// Creates an orthographic projection matrix
template <typename T>
Matrix4x4<T> CreateOrthographicMatrix(
	T width, T height,
	T nearPlaneDistance, T farPlaneDistance);
	
// Creates an orthographic projection matrix with off-center parameters
template <typename T>
Matrix4x4<T> CreateOrthographicOffCenterMatrix(
	T left, T right,
	T bottom, T top,
	T nearPlaneDistance, T farPlaneDistance);

// Creates a view matrix for VR (for specific eye)
template <typename T>
Matrix4x4<T> CreateViewMatrixForEye(
	const Vector3<T>& headPosition,
	const Quaternion& headOrientation,
	const Vector3<T>& eyeOffset);

// Creates a world matrix from position, rotation, and scale
template <typename T>
Matrix4x4<T> CreateWorldMatrix(
	const Vector3<T>& position,
	const Quaternion& rotation,
	const Vector3<T>& scale);

//------------------------------------------------------------------------------
// Matrix Decomposition
//------------------------------------------------------------------------------

// Decomposes a matrix into scale, rotation, and translation components
template <typename T>
bool DecomposeMatrix(
	const Matrix4x4<T>& matrix,
	Vector3<T>& scale,
	Quaternion& rotation,
	Vector3<T>& translation);

// Decomposes a matrix into affine components and validates if it's valid
template <typename T>
bool DecomposeAffineMatrix(
	const Matrix4x4<T>& matrix,
	Vector3<T>& scale,
	Matrix4x4<T>& rotationShear,
	Vector3<T>& translation);

//------------------------------------------------------------------------------
// Matrix Analysis
//------------------------------------------------------------------------------

// Checks if the matrix is identity
template <typename T>
bool IsIdentity(const Matrix4x4<T>& matrix);

// Checks if a matrix is valid (no infinities, NaN, etc.)
template <typename T>
bool IsValid(const Matrix4x4<T>& matrix);

// Computes the condition number of a matrix (a measure of numerical stability)
template <typename T>
T ConditionNumber(const Matrix4x4<T>& matrix);

// Computes the norm of a matrix
template <typename T>
T MatrixNorm(const Matrix4x4<T>& matrix, int normType = 2);

//------------------------------------------------------------------------------
// Matrix Factorization
//------------------------------------------------------------------------------

// LU decomposition with partial pivoting
template <typename T>
bool LUDecomposition(
	const Matrix4x4<T>& matrix,
	Matrix4x4<T>& lower,
	Matrix4x4<T>& upper,
	Matrix4x4<T>& permutation);

// QR decomposition 
template <typename T>
bool QRDecomposition(
	const Matrix4x4<T>& matrix,
	Matrix4x4<T>& q,
	Matrix4x4<T>& r);

// Singular Value Decomposition (SVD)
template <typename T>
bool SVDecomposition(
	const Matrix4x4<T>& matrix,
	Matrix4x4<T>& u,
	Vector4<T>& sigma,
	Matrix4x4<T>& v);

//------------------------------------------------------------------------------
// Matrix Operations
//------------------------------------------------------------------------------

// Computes the adjugate of a matrix
template <typename T>
Matrix4x4<T> Adjugate(const Matrix4x4<T>& matrix);

// Computes the cofactor matrix
template <typename T>
Matrix4x4<T> CofactorMatrix(const Matrix4x4<T>& matrix);

// Computes the trace of a matrix (sum of diagonal elements)
template <typename T>
T Trace(const Matrix4x4<T>& matrix);

// Computes the sum of all matrix elements
template <typename T>
T Sum(const Matrix4x4<T>& matrix);

// Linearizes a transformation matrix (removes perspective components)
template <typename T>
Matrix4x4<T> Linearize(const Matrix4x4<T>& projectionMatrix);

// Extracts the frustum planes from a view-projection matrix
template <typename T>
void ExtractFrustumPlanes(
	const Matrix4x4<T>& viewProjection,
	Vector4<T>& left,
	Vector4<T>& right,
	Vector4<T>& bottom,
	Vector4<T>& top,
	Vector4<T>& near,
	Vector4<T>& far);

// Transforms a frustum by a matrix
template <typename T>
void TransformFrustum(
	const Matrix4x4<T>& transform,
	Vector4<T>& left,
	Vector4<T>& right,
	Vector4<T>& bottom,
	Vector4<T>& top,
	Vector4<T>& near,
	Vector4<T>& far);

// Specialized 3x3 matrix inversion for rotation matrices (more efficient than general inversion)
template <typename T>
Matrix3x3<T> InvertRotationMatrix(const Matrix3x3<T>& rotationMatrix);

// Transform a bounding box by a matrix
template <typename T>
void TransformBoundingBox(
	const Matrix4x4<T>& matrix,
	const Vector3<T>& min,
	const Vector3<T>& max,
	Vector3<T>& newMin,
	Vector3<T>& newMax);

// Type definitions for common matrix types
using Mat3f = Matrix3x3<float>;
using Mat4f = Matrix4x4<float>;
using Mat3d = Matrix3x3<double>;
using Mat4d = Matrix4x4<double>;

} // namespace math
} // namespace glib

// Include inline implementations
#include "MatrixMath.inl"