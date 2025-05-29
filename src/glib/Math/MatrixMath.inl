#pragma once

#include "MatrixMath.h"

namespace glib {
namespace math {

//------------------------------------------------------------------------------
// Matrix Transformation Utilities
//------------------------------------------------------------------------------

template <typename T>
Matrix4x4<T> CreateBillboardMatrix(
	const Vector3<T>& objectPosition,
	const Vector3<T>& cameraPosition,
	const Vector3<T>& cameraUp,
	const Vector3<T>* cameraForward)
{
	Vector3<T> zaxis;
	
	if (cameraForward) {
		zaxis = -*cameraForward;
	} else {
		zaxis = (objectPosition - cameraPosition).Normalized();
	}

	// Handle case when object is directly behind camera
	if (zaxis.LengthSquared() < EPSILON) {
		zaxis = Vector3<T>(0, 0, -1);
	}

	Vector3<T> xaxis = cameraUp.Cross(zaxis).Normalized();
	
	// Handle case when camera up is parallel to z axis
	if (xaxis.LengthSquared() < EPSILON) {
		xaxis = Vector3<T>(1, 0, 0);
	}

	Vector3<T> yaxis = zaxis.Cross(xaxis);

	// Create rotation matrix
	Matrix4x4<T> result;
	result(0, 0) = xaxis.x;
	result(0, 1) = xaxis.y;
	result(0, 2) = xaxis.z;
	result(0, 3) = 0;
	
	result(1, 0) = yaxis.x;
	result(1, 1) = yaxis.y;
	result(1, 2) = yaxis.z;
	result(1, 3) = 0;
	
	result(2, 0) = zaxis.x;
	result(2, 1) = zaxis.y;
	result(2, 2) = zaxis.z;
	result(2, 3) = 0;
	
	result(3, 0) = objectPosition.x;
	result(3, 1) = objectPosition.y;
	result(3, 2) = objectPosition.z;
	result(3, 3) = 1;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreateConstrainedBillboardMatrix(
	const Vector3<T>& objectPosition,
	const Vector3<T>& cameraPosition,
	const Vector3<T>& rotateAxis,
	const Vector3<T>* cameraForward,
	const Vector3<T>* objectForward)
{
	Vector3<T> faceDir;
	
	if (cameraForward) {
		faceDir = -*cameraForward;
	} else {
		faceDir = (objectPosition - cameraPosition).Normalized();
	}

	// Make sure rotate axis is normalized
	Vector3<T> normRotateAxis = rotateAxis.Normalized();

	// Check if face direction is parallel to rotate axis
	T dot = normRotateAxis.Dot(faceDir);
	
	// If they're nearly parallel, use the object's forward direction
	if (std::abs(dot) > 0.9982f) {  // ~3.5 degrees
		if (objectForward) {
			Vector3<T> objectUp = *objectForward;
			
			// If object forward and rotate axis are parallel, use another axis
			dot = normRotateAxis.Dot(objectUp);
			
			if (std::abs(dot) > 0.9982f) {
				// Use arbitrary up vector perpendicular to rotate axis
				objectUp = std::abs(normRotateAxis.z) > 0.9982f ?
					Vector3<T>(1, 0, 0) : Vector3<T>(0, 0, 1);
			}
			
			// Create a vector perpendicular to rotate axis and object forward
			faceDir = normRotateAxis.Cross(objectUp).Normalized();
		}
		else {
			// Default fallback
			faceDir = std::abs(normRotateAxis.z) > 0.9982f ?
				Vector3<T>(1, 0, 0) : Vector3<T>(0, 0, 1);
		}
	}

	// Create x and y axes perpendicular to rotate axis
	Vector3<T> xaxis = normRotateAxis.Cross(faceDir).Normalized();
	Vector3<T> yaxis = normRotateAxis.Cross(xaxis);

	// Create rotation matrix
	Matrix4x4<T> result;
	result(0, 0) = xaxis.x;
	result(0, 1) = xaxis.y;
	result(0, 2) = xaxis.z;
	result(0, 3) = 0;
	
	result(1, 0) = normRotateAxis.x;
	result(1, 1) = normRotateAxis.y;
	result(1, 2) = normRotateAxis.z;
	result(1, 3) = 0;
	
	result(2, 0) = yaxis.x;
	result(2, 1) = yaxis.y;
	result(2, 2) = yaxis.z;
	result(2, 3) = 0;
	
	result(3, 0) = objectPosition.x;
	result(3, 1) = objectPosition.y;
	result(3, 2) = objectPosition.z;
	result(3, 3) = 1;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreateLookAtMatrix(
	const Vector3<T>& cameraPosition,
	const Vector3<T>& cameraTarget,
	const Vector3<T>& cameraUpVector)
{
	// This is equivalent to the Matrix4x4<T>::LookAt method
	// but included here for completeness
	Vector3<T> zaxis = (cameraPosition - cameraTarget).Normalized();
	Vector3<T> xaxis = cameraUpVector.Cross(zaxis).Normalized();
	Vector3<T> yaxis = zaxis.Cross(xaxis);
	
	Matrix4x4<T> result;
	result(0, 0) = xaxis.x;
	result(0, 1) = yaxis.x;
	result(0, 2) = zaxis.x;
	result(0, 3) = 0;
	
	result(1, 0) = xaxis.y;
	result(1, 1) = yaxis.y;
	result(1, 2) = zaxis.y;
	result(1, 3) = 0;
	
	result(2, 0) = xaxis.z;
	result(2, 1) = yaxis.z;
	result(2, 2) = zaxis.z;
	result(2, 3) = 0;
	
	result(3, 0) = -xaxis.Dot(cameraPosition);
	result(3, 1) = -yaxis.Dot(cameraPosition);
	result(3, 2) = -zaxis.Dot(cameraPosition);
	result(3, 3) = 1;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreatePerspectiveMatrix(
	T fieldOfView,
	T aspectRatio,
	T nearPlaneDistance,
	T farPlaneDistance)
{
	assert(nearPlaneDistance > 0 && farPlaneDistance > nearPlaneDistance);
	T yScale = 1.0 / std::tan(fieldOfView * 0.5);
	T xScale = yScale / aspectRatio;
	
	Matrix4x4<T> result;
	result(0, 0) = xScale;
	result(0, 1) = 0;
	result(0, 2) = 0;
	result(0, 3) = 0;
	
	result(1, 0) = 0;
	result(1, 1) = yScale;
	result(1, 2) = 0;
	result(1, 3) = 0;
	
	result(2, 0) = 0;
	result(2, 1) = 0;
	result(2, 2) = farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
	result(2, 3) = -1;
	
	result(3, 0) = 0;
	result(3, 1) = 0;
	result(3, 2) = nearPlaneDistance * farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
	result(3, 3) = 0;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreatePerspectiveOffCenterMatrix(
	T left, T right, 
	T bottom, T top,
	T nearPlaneDistance, T farPlaneDistance)
{
	assert(nearPlaneDistance > 0 && farPlaneDistance > nearPlaneDistance);
	
	Matrix4x4<T> result;
	result(0, 0) = 2 * nearPlaneDistance / (right - left);
	result(0, 1) = 0;
	result(0, 2) = (right + left) / (right - left);
	result(0, 3) = 0;
	
	result(1, 0) = 0;
	result(1, 1) = 2 * nearPlaneDistance / (top - bottom);
	result(1, 2) = (top + bottom) / (top - bottom);
	result(1, 3) = 0;
	
	result(2, 0) = 0;
	result(2, 1) = 0;
	result(2, 2) = farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
	result(2, 3) = -1;
	
	result(3, 0) = 0;
	result(3, 1) = 0;
	result(3, 2) = nearPlaneDistance * farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
	result(3, 3) = 0;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreateOrthographicMatrix(
	T width, T height,
	T nearPlaneDistance, T farPlaneDistance)
{
	T halfWidth = width * 0.5;
	T halfHeight = height * 0.5;
	
	return CreateOrthographicOffCenterMatrix(
		-halfWidth, halfWidth,
		-halfHeight, halfHeight,
		nearPlaneDistance, farPlaneDistance);
}

template <typename T>
Matrix4x4<T> CreateOrthographicOffCenterMatrix(
	T left, T right,
	T bottom, T top,
	T nearPlaneDistance, T farPlaneDistance)
{
	T recipWidth = 1 / (right - left);
	T recipHeight = 1 / (top - bottom);
	T rangeZ = 1 / (farPlaneDistance - nearPlaneDistance);
	
	Matrix4x4<T> result;
	result(0, 0) = 2 * recipWidth;
	result(0, 1) = 0;
	result(0, 2) = 0;
	result(0, 3) = 0;
	
	result(1, 0) = 0;
	result(1, 1) = 2 * recipHeight;
	result(1, 2) = 0;
	result(1, 3) = 0;
	
	result(2, 0) = 0;
	result(2, 1) = 0;
	result(2, 2) = rangeZ;
	result(2, 3) = 0;
	
	result(3, 0) = -(right + left) * recipWidth;
	result(3, 1) = -(top + bottom) * recipHeight;
	result(3, 2) = -nearPlaneDistance * rangeZ;
	result(3, 3) = 1;
	
	return result;
}

template <typename T>
Matrix4x4<T> CreateViewMatrixForEye(
	const Vector3<T>& headPosition,
	const Quaternion& headOrientation,
	const Vector3<T>& eyeOffset)
{
	// Create rotation matrix from head orientation
	Matrix4x4<T> rotationMatrix = headOrientation.ToMatrix4();
	
	// Apply eye offset in head space
	Vector3<T> eyePosition = headPosition + rotationMatrix * eyeOffset;
	
	// Extract view forward direction from rotation matrix (negative z-axis)
	Vector3<T> viewForward(-rotationMatrix(0, 2), -rotationMatrix(1, 2), -rotationMatrix(2, 2));
	
	// Extract up direction from rotation matrix (y-axis)
	Vector3<T> viewUp(rotationMatrix(0, 1), rotationMatrix(1, 1), rotationMatrix(2, 1));
	
	// Create view matrix for this eye position
	return CreateLookAtMatrix(eyePosition, eyePosition + viewForward, viewUp);
}

template <typename T>
Matrix4x4<T> CreateWorldMatrix(
	const Vector3<T>& position,
	const Quaternion& rotation,
	const Vector3<T>& scale)
{
	Matrix4x4<T> scaleMatrix = Matrix4x4<T>::Scaling(scale);
	Matrix4x4<T> rotationMatrix = rotation.ToMatrix4();
	Matrix4x4<T> translationMatrix = Matrix4x4<T>::Translation(position);
	
	// Combine: SRT (Scale, Rotate, Translate)
	return scaleMatrix * rotationMatrix * translationMatrix;
}

//------------------------------------------------------------------------------
// Matrix Decomposition
//------------------------------------------------------------------------------

template <typename T>
bool DecomposeMatrix(
	const Matrix4x4<T>& matrix,
	Vector3<T>& scale,
	Quaternion& rotation,
	Vector3<T>& translation)
{
	// Extract translation
	translation.x = matrix(3, 0);
	translation.y = matrix(3, 1);
	translation.z = matrix(3, 2);
	
	// Extract scale
	// Use length of first three rows as scale factors
	Vector3<T> row0(matrix(0, 0), matrix(0, 1), matrix(0, 2));
	Vector3<T> row1(matrix(1, 0), matrix(1, 1), matrix(1, 2));
	Vector3<T> row2(matrix(2, 0), matrix(2, 1), matrix(2, 2));
	
	scale.x = row0.Length();
	scale.y = row1.Length();
	scale.z = row2.Length();
	
	// If any scale is zero, we can't extract rotation
	if (scale.x < EPSILON || scale.y < EPSILON || scale.z < EPSILON) {
		rotation = Quaternion::Identity();
		return false;
	}
	
	// Normalize the rotation matrix
	Matrix3x3<T> rotationMat(
		matrix(0, 0) / scale.x, matrix(0, 1) / scale.x, matrix(0, 2) / scale.x,
		matrix(1, 0) / scale.y, matrix(1, 1) / scale.y, matrix(1, 2) / scale.y,
		matrix(2, 0) / scale.z, matrix(2, 1) / scale.z, matrix(2, 2) / scale.z
	);
	
	// Convert to quaternion
	rotation = Quaternion(rotationMat);
	
	return true;
}

template <typename T>
bool DecomposeAffineMatrix(
	const Matrix4x4<T>& matrix,
	Vector3<T>& scale,
	Matrix4x4<T>& rotationShear,
	Vector3<T>& translation)
{
	// Extract translation
	translation.x = matrix(3, 0);
	translation.y = matrix(3, 1);
	translation.z = matrix(3, 2);
	
	// Extract the upper 3x3 matrix
	Matrix3x3<T> upperLeft(
		matrix(0, 0), matrix(0, 1), matrix(0, 2),
		matrix(1, 0), matrix(1, 1), matrix(1, 2),
		matrix(2, 0), matrix(2, 1), matrix(2, 2)
	);
	
	// Use QR decomposition: A = QR, where Q is orthogonal and R is upper triangular
	Matrix3x3<T> q, r;
	if (!QRDecomposition(upperLeft, q, r)) {
		return false;
	}
	
	// Scale factors are on the diagonal of R
	scale.x = r(0, 0);
	scale.y = r(1, 1);
	scale.z = r(2, 2);
	
	// If any scale is zero, we can't continue
	if (std::abs(scale.x) < EPSILON || 
		std::abs(scale.y) < EPSILON || 
		std::abs(scale.z) < EPSILON) {
		return false;
	}
	
	// Normalize R to get shear components
	r(0, 0) = 1.0;
	r(1, 1) = 1.0;
	r(2, 2) = 1.0;
	
	// Combine rotation and shear into a 4x4 matrix
	rotationShear = Matrix4x4<T>::Identity();
	
	// Upper 3x3 contains rotation*shear (Q*R)
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rotationShear(i, j) = upperLeft(i, j);
		}
	}
	
	return true;
}

//------------------------------------------------------------------------------
// Matrix Analysis
//------------------------------------------------------------------------------

template <typename T>
bool IsIdentity(const Matrix4x4<T>& matrix)
{
	static const T threshold = static_cast<T>(EPSILON);
	
	return 
		std::abs(matrix(0, 0) - 1) < threshold &&
		std::abs(matrix(1, 1) - 1) < threshold &&
		std::abs(matrix(2, 2) - 1) < threshold &&
		std::abs(matrix(3, 3) - 1) < threshold &&
		
		std::abs(matrix(0, 1)) < threshold &&
		std::abs(matrix(0, 2)) < threshold &&
		std::abs(matrix(0, 3)) < threshold &&
		
		std::abs(matrix(1, 0)) < threshold &&
		std::abs(matrix(1, 2)) < threshold &&
		std::abs(matrix(1, 3)) < threshold &&
		
		std::abs(matrix(2, 0)) < threshold &&
		std::abs(matrix(2, 1)) < threshold &&
		std::abs(matrix(2, 3)) < threshold &&
		
		std::abs(matrix(3, 0)) < threshold &&
		std::abs(matrix(3, 1)) < threshold &&
		std::abs(matrix(3, 2)) < threshold;
}

template <typename T>
bool IsValid(const Matrix4x4<T>& matrix)
{
	// Check for NaN or Infinity
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			T val = matrix(i, j);
			if (std::isnan(val) || std::isinf(val)) {
				return false;
			}
		}
	}
	return true;
}

template <typename T>
T MatrixNorm(const Matrix4x4<T>& matrix, int normType)
{
	switch (normType) {
		case 1: {  // L1 norm (maximum column sum)
			T maxColSum = 0;
			for (int j = 0; j < 4; j++) {
				T colSum = std::abs(matrix(0, j)) +
						  std::abs(matrix(1, j)) +
						  std::abs(matrix(2, j)) +
						  std::abs(matrix(3, j));
				maxColSum = std::max(maxColSum, colSum);
			}
			return maxColSum;
		}
		
		case 2: {  // L2 norm (spectral norm) - approximation
			// Not computing singular values here, just approximation
			T sum = 0;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					sum += matrix(i, j) * matrix(i, j);
				}
			}
			return std::sqrt(sum);
		}
		
		default:
		case 0: {  // Frobenius norm
			T sum = 0;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					sum += matrix(i, j) * matrix(i, j);
				}
			}
			return std::sqrt(sum);
		}
	}
}

template <typename T>
T ConditionNumber(const Matrix4x4<T>& matrix)
{
	// This is a simplified estimation - for proper condition number
	// we would need SVD to get max/min singular values
	
	// Condition number = ||A|| * ||A^-1||
	Matrix4x4<T> inverse;
	try {
		inverse = matrix.Inverse();
	} catch (...) {
		// Matrix is singular, condition number is infinity
		return std::numeric_limits<T>::infinity();
	}
	
	T normA = MatrixNorm(matrix, 1);
	T normAInv = MatrixNorm(inverse, 1);
	
	return normA * normAInv;
}

//------------------------------------------------------------------------------
// Matrix Operations
//------------------------------------------------------------------------------

template <typename T>
T Trace(const Matrix4x4<T>& matrix)
{
	return matrix(0, 0) + matrix(1, 1) + matrix(2, 2) + matrix(3, 3);
}

template <typename T>
T Sum(const Matrix4x4<T>& matrix)
{
	T result = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result += matrix(i, j);
		}
	}
	return result;
}

template <typename T>
Matrix4x4<T> Linearize(const Matrix4x4<T>& projectionMatrix)
{
	Matrix4x4<T> result = projectionMatrix;
	
	// Remove perspective components
	result(0, 3) = 0;
	result(1, 3) = 0;
	result(2, 3) = 0;
	result(3, 3) = 1;
	
	return result;
}

template <typename T>
void ExtractFrustumPlanes(
	const Matrix4x4<T>& viewProjection,
	Vector4<T>& left,
	Vector4<T>& right,
	Vector4<T>& bottom,
	Vector4<T>& top,
	Vector4<T>& near,
	Vector4<T>& far)
{
	// Left plane
	left.x = viewProjection(0, 3) + viewProjection(0, 0);
	left.y = viewProjection(1, 3) + viewProjection(1, 0);
	left.z = viewProjection(2, 3) + viewProjection(2, 0);
	left.w = viewProjection(3, 3) + viewProjection(3, 0);
	
	// Right plane
	right.x = viewProjection(0, 3) - viewProjection(0, 0);
	right.y = viewProjection(1, 3) - viewProjection(1, 0);
	right.z = viewProjection(2, 3) - viewProjection(2, 0);
	right.w = viewProjection(3, 3) - viewProjection(3, 0);
	
	// Bottom plane
	bottom.x = viewProjection(0, 3) + viewProjection(0, 1);
	bottom.y = viewProjection(1, 3) + viewProjection(1, 1);
	bottom.z = viewProjection(2, 3) + viewProjection(2, 1);
	bottom.w = viewProjection(3, 3) + viewProjection(3, 1);
	
	// Top plane
	top.x = viewProjection(0, 3) - viewProjection(0, 1);
	top.y = viewProjection(1, 3) - viewProjection(1, 1);
	top.z = viewProjection(2, 3) - viewProjection(2, 1);
	top.w = viewProjection(3, 3) - viewProjection(3, 1);
	
	// Near plane
	near.x = viewProjection(0, 2);
	near.y = viewProjection(1, 2);
	near.z = viewProjection(2, 2);
	near.w = viewProjection(3, 2);
	
	// Far plane
	far.x = viewProjection(0, 3) - viewProjection(0, 2);
	far.y = viewProjection(1, 3) - viewProjection(1, 2);
	far.z = viewProjection(2, 3) - viewProjection(2, 2);
	far.w = viewProjection(3, 3) - viewProjection(3, 2);
	
	// Normalize planes
	T invLen = 1.0 / sqrt(left.x * left.x + left.y * left.y + left.z * left.z);
	left *= invLen;
	
	invLen = 1.0 / sqrt(right.x * right.x + right.y * right.y + right.z * right.z);
	right *= invLen;
	
	invLen = 1.0 / sqrt(bottom.x * bottom.x + bottom.y * bottom.y + bottom.z * bottom.z);
	bottom *= invLen;
	
	invLen = 1.0 / sqrt(top.x * top.x + top.y * top.y + top.z * top.z);
	top *= invLen;
	
	invLen = 1.0 / sqrt(near.x * near.x + near.y * near.y + near.z * near.z);
	near *= invLen;
	
	invLen = 1.0 / sqrt(far.x * far.x + far.y * far.y + far.z * far.z);
	far *= invLen;
}

template <typename T>
void TransformFrustum(
	const Matrix4x4<T>& transform,
	Vector4<T>& left,
	Vector4<T>& right,
	Vector4<T>& bottom,
	Vector4<T>& top,
	Vector4<T>& near,
	Vector4<T>& far)
{
	// Calculate inverse transpose for transforming planes
	Matrix4x4<T> invTranspose = transform.Inverse().Transposed();
	
	// Transform each frustum plane
	left = invTranspose * left;
	right = invTranspose * right;
	bottom = invTranspose * bottom;
	top = invTranspose * top;
	near = invTranspose * near;
	far = invTranspose * far;
	
	// Re-normalize the planes
	T invLen = 1.0 / sqrt(left.x * left.x + left.y * left.y + left.z * left.z);
	left *= invLen;
	
	invLen = 1.0 / sqrt(right.x * right.x + right.y * right.y + right.z * right.z);
	right *= invLen;
	
	invLen = 1.0 / sqrt(bottom.x * bottom.x + bottom.y * bottom.y + bottom.z * bottom.z);
	bottom *= invLen;
	
	invLen = 1.0 / sqrt(top.x * top.x + top.y * top.y + top.z * top.z);
	top *= invLen;
	
	invLen = 1.0 / sqrt(near.x * near.x + near.y * near.y + near.z * near.z);
	near *= invLen;
	
	invLen = 1.0 / sqrt(far.x * far.x + far.y * far.y + far.z * far.z);
	far *= invLen;
}

template <typename T>
Matrix3x3<T> InvertRotationMatrix(const Matrix3x3<T>& rotationMatrix)
{
	// Assuming input is a pure rotation matrix, the inverse is simply the transpose
	return rotationMatrix.Transposed();
}

template <typename T>
void TransformBoundingBox(
	const Matrix4x4<T>& matrix,
	const Vector3<T>& min,
	const Vector3<T>& max,
	Vector3<T>& newMin,
	Vector3<T>& newMax)
{
	// Initialize with transformed center point
	Vector3<T> center = (min + max) * 0.5f;
	Vector4<T> transformedCenter = matrix * Vector4<T>(center.x, center.y, center.z, 1.0);
	
	// If w is 0, use a small epsilon to avoid division by zero
	T w = (std::abs(transformedCenter.w) < EPSILON) ? EPSILON : transformedCenter.w;
	
	newMin.x = transformedCenter.x / w;
	newMin.y = transformedCenter.y / w;
	newMin.z = transformedCenter.z / w;
	
	newMax = newMin;
	
	// Calculate extents
	Vector3<T> extents = (max - min) * 0.5f;
	
	// For each axis
	for (int i = 0; i < 3; i++) {
		// For each corner of the box
		for (int j = 0; j < 8; j++) {
			// Calculate transformed point
			Vector3<T> corner = center;
			corner.x += (j & 1) ? extents.x : -extents.x;
			corner.y += (j & 2) ? extents.y : -extents.y;
			corner.z += (j & 4) ? extents.z : -extents.z;
			
			// Transform point
			Vector4<T> transformedCorner = matrix * Vector4<T>(corner.x, corner.y, corner.z, 1.0);
			
			// Handle perspective division
			w = (std::abs(transformedCorner.w) < EPSILON) ? EPSILON : transformedCorner.w;
			Vector3<T> projectedCorner(
				transformedCorner.x / w,
				transformedCorner.y / w,
				transformedCorner.z / w
			);
			
			// Update bounds
			newMin.x = std::min(newMin.x, projectedCorner.x);
			newMin.y = std::min(newMin.y, projectedCorner.y);
			newMin.z = std::min(newMin.z, projectedCorner.z);
			
			newMax.x = std::max(newMax.x, projectedCorner.x);
			newMax.y = std::max(newMax.y, projectedCorner.y);
			newMax.z = std::max(newMax.z, projectedCorner.z);
		}
	}
}

template <typename T>
Matrix4x4<T> Adjugate(const Matrix4x4<T>& matrix)
{
	// Calculate cofactor matrix
	Matrix4x4<T> cofactor = CofactorMatrix(matrix);
	
	// Return transpose of cofactor matrix
	return cofactor.Transposed();
}

template <typename T>
Matrix4x4<T> CofactorMatrix(const Matrix4x4<T>& matrix)
{
	Matrix4x4<T> result;
	
	// Calculate minors and cofactors
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			// Calculate minor by eliminating row i and column j
			T minor = 0;
			int rows[3], cols[3];
			
			// Get rows except i
			for (int r = 0, index = 0; r < 4; r++) {
				if (r != i) {
					rows[index++] = r;
				}
			}
			
			// Get columns except j
			for (int c = 0, index = 0; c < 4; c++) {
				if (c != j) {
					cols[index++] = c;
				}
			}
			
			// Calculate 3x3 determinant for minor
			minor = 
				matrix(rows[0], cols[0]) * (matrix(rows[1], cols[1]) * matrix(rows[2], cols[2]) - matrix(rows[1], cols[2]) * matrix(rows[2], cols[1])) -
				matrix(rows[0], cols[1]) * (matrix(rows[1], cols[0]) * matrix(rows[2], cols[2]) - matrix(rows[1], cols[2]) * matrix(rows[2], cols[0])) +
				matrix(rows[0], cols[2]) * (matrix(rows[1], cols[0]) * matrix(rows[2], cols[1]) - matrix(rows[1], cols[1]) * matrix(rows[2], cols[0]));
			
			// Apply checkerboard pattern for cofactor
			result(i, j) = ((i + j) % 2 == 0) ? minor : -minor;
		}
	}
	
	return result;
}

} // namespace math
} // namespace glib