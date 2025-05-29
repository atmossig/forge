#pragma once

#include <cmath>
#include <iostream>
#include <cassert>

namespace glib {
namespace math {

// Constants
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = PI * 2.0f;
constexpr float HALF_PI = PI * 0.5f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float EPSILON = 0.000001f;

// Utility functions
inline bool NearlyEqual(float a, float b, float epsilon = EPSILON) {
	return std::abs(a - b) <= epsilon;
}

inline float Lerp(float a, float b, float t) {
	return a + t * (b - a);
}

inline float Clamp(float value, float min, float max) {
	return (value < min) ? min : (value > max) ? max : value;
}

// Forward declarations
template <typename T> class Vector2;
template <typename T> class Vector3;
template <typename T> class Vector4;
template <typename T> class Matrix3x3;
template <typename T> class Matrix4x4;
class Quaternion;

// Vector2 class
template <typename T>
class Vector2 {
public:
	T x, y;

	Vector2() : x(0), y(0) {}
	Vector2(T x, T y) : x(x), y(y) {}
	explicit Vector2(T scalar) : x(scalar), y(scalar) {}

	// Operators
	Vector2 operator+(const Vector2& rhs) const;
	Vector2 operator-(const Vector2& rhs) const;
	Vector2 operator*(T scalar) const;
	Vector2 operator/(T scalar) const;
	
	Vector2& operator+=(const Vector2& rhs);
	Vector2& operator-=(const Vector2& rhs);
	Vector2& operator*=(T scalar);
	Vector2& operator/=(T scalar);
	
	bool operator==(const Vector2& rhs) const;
	bool operator!=(const Vector2& rhs) const;
	
	// Math operations
	T Length() const;
	T LengthSquared() const;
	Vector2 Normalized() const;
	void Normalize();
	T Dot(const Vector2& rhs) const;
	T Cross(const Vector2& rhs) const;  // 2D "cross product" returns a scalar
	
	// Static helper methods
	static Vector2 Zero() { return Vector2(0, 0); }
	static Vector2 One() { return Vector2(1, 1); }
	static Vector2 UnitX() { return Vector2(1, 0); }
	static Vector2 UnitY() { return Vector2(0, 1); }
	
	static T Distance(const Vector2& a, const Vector2& b);
	static T DistanceSquared(const Vector2& a, const Vector2& b);
	static Vector2 Lerp(const Vector2& a, const Vector2& b, T t);
};

// Vector3 class
template <typename T>
class Vector3 {
public:
	T x, y, z;

	Vector3() : x(0), y(0), z(0) {}
	Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
	explicit Vector3(T scalar) : x(scalar), y(scalar), z(scalar) {}
	
	// Conversion from Vector2
	explicit Vector3(const Vector2<T>& v, T z = 0) : x(v.x), y(v.y), z(z) {}

	// Operators
	Vector3 operator+(const Vector3& rhs) const;
	Vector3 operator-(const Vector3& rhs) const;
	Vector3 operator*(T scalar) const;
	Vector3 operator/(T scalar) const;
	
	Vector3& operator+=(const Vector3& rhs);
	Vector3& operator-=(const Vector3& rhs);
	Vector3& operator*=(T scalar);
	Vector3& operator/=(T scalar);
	
	bool operator==(const Vector3& rhs) const;
	bool operator!=(const Vector3& rhs) const;
	
	// Math operations
	T Length() const;
	T LengthSquared() const;
	Vector3 Normalized() const;
	void Normalize();
	T Dot(const Vector3& rhs) const;
	Vector3 Cross(const Vector3& rhs) const;
	
	// Static helper methods
	static Vector3 Zero() { return Vector3(0, 0, 0); }
	static Vector3 One() { return Vector3(1, 1, 1); }
	static Vector3 UnitX() { return Vector3(1, 0, 0); }
	static Vector3 UnitY() { return Vector3(0, 1, 0); }
	static Vector3 UnitZ() { return Vector3(0, 0, 1); }
	
	static T Distance(const Vector3& a, const Vector3& b);
	static T DistanceSquared(const Vector3& a, const Vector3& b);
	static Vector3 Lerp(const Vector3& a, const Vector3& b, T t);
	static Vector3 Reflect(const Vector3& direction, const Vector3& normal);
};

// Vector4 class
template <typename T>
class Vector4 {
public:
	T x, y, z, w;

	Vector4() : x(0), y(0), z(0), w(0) {}
	Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
	explicit Vector4(T scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}
	
	// Conversions
	explicit Vector4(const Vector3<T>& v, T w = 1) : x(v.x), y(v.y), z(v.z), w(w) {}
	explicit Vector4(const Vector2<T>& v, T z = 0, T w = 1) : x(v.x), y(v.y), z(z), w(w) {}

	// Operators
	Vector4 operator+(const Vector4& rhs) const;
	Vector4 operator-(const Vector4& rhs) const;
	Vector4 operator*(T scalar) const;
	Vector4 operator/(T scalar) const;
	
	Vector4& operator+=(const Vector4& rhs);
	Vector4& operator-=(const Vector4& rhs);
	Vector4& operator*=(T scalar);
	Vector4& operator/=(T scalar);
	
	bool operator==(const Vector4& rhs) const;
	bool operator!=(const Vector4& rhs) const;
	
	// Math operations
	T Length() const;
	T LengthSquared() const;
	Vector4 Normalized() const;
	void Normalize();
	T Dot(const Vector4& rhs) const;
	
	// Static helper methods
	static Vector4 Zero() { return Vector4(0, 0, 0, 0); }
	static Vector4 One() { return Vector4(1, 1, 1, 1); }
	static Vector4 UnitX() { return Vector4(1, 0, 0, 0); }
	static Vector4 UnitY() { return Vector4(0, 1, 0, 0); }
	static Vector4 UnitZ() { return Vector4(0, 0, 1, 0); }
	static Vector4 UnitW() { return Vector4(0, 0, 0, 1); }
	
	static T Distance(const Vector4& a, const Vector4& b);
	static T DistanceSquared(const Vector4& a, const Vector4& b);
	static Vector4 Lerp(const Vector4& a, const Vector4& b, T t);
};

// Matrix3x3 class (primarily for 2D transformations)
template <typename T>
class Matrix3x3 {
public:
	// Stored in row-major order
	T m[9];
	
	Matrix3x3();
	Matrix3x3(T m00, T m01, T m02,
			  T m10, T m11, T m12,
			  T m20, T m21, T m22);
	
	// Operators
	Matrix3x3 operator*(const Matrix3x3& rhs) const;
	Vector3<T> operator*(const Vector3<T>& vec) const;
	Matrix3x3& operator*=(const Matrix3x3& rhs);
	
	T& operator()(int row, int col);
	T operator()(int row, int col) const;
	
	// Transformations
	static Matrix3x3 Identity();
	static Matrix3x3 Translation(T x, T y);
	static Matrix3x3 Translation(const Vector2<T>& translation);
	static Matrix3x3 Rotation(T angleInRadians);
	static Matrix3x3 Scaling(T x, T y);
	static Matrix3x3 Scaling(const Vector2<T>& scale);
	
	// Operations
	Matrix3x3 Transposed() const;
	T Determinant() const;
	Matrix3x3 Inverse() const;
	bool Invert();
};

// Matrix4x4 class (primarily for 3D transformations)
template <typename T>
class Matrix4x4 {
public:
	// Stored in row-major order
	T m[16];
	
	Matrix4x4();
	Matrix4x4(T m00, T m01, T m02, T m03,
			  T m10, T m11, T m12, T m13,
			  T m20, T m21, T m22, T m23,
			  T m30, T m31, T m32, T m33);
	
	// Operators
	Matrix4x4 operator*(const Matrix4x4& rhs) const;
	Vector4<T> operator*(const Vector4<T>& vec) const;
	Matrix4x4& operator*=(const Matrix4x4& rhs);
	
	T& operator()(int row, int col);
	T operator()(int row, int col) const;
	
	// Transformations
	static Matrix4x4 Identity();
	static Matrix4x4 Translation(T x, T y, T z);
	static Matrix4x4 Translation(const Vector3<T>& translation);
	static Matrix4x4 RotationX(T angleInRadians);
	static Matrix4x4 RotationY(T angleInRadians);
	static Matrix4x4 RotationZ(T angleInRadians);
	static Matrix4x4 Rotation(const Vector3<T>& axis, T angleInRadians);
	static Matrix4x4 Scaling(T x, T y, T z);
	static Matrix4x4 Scaling(const Vector3<T>& scale);
	
	// Projection matrices
	static Matrix4x4 Perspective(T fovY, T aspectRatio, T nearZ, T farZ);
	static Matrix4x4 Orthographic(T left, T right, T bottom, T top, T nearZ, T farZ);
	static Matrix4x4 LookAt(const Vector3<T>& eye, const Vector3<T>& target, const Vector3<T>& up);
	
	// Operations
	Matrix4x4 Transposed() const;
	T Determinant() const;
	Matrix4x4 Inverse() const;
	bool Invert();
};

// Quaternion class
class Quaternion {
public:
	float x, y, z, w;
	
	Quaternion() : x(0), y(0), z(0), w(1) {}
	Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	
	// Conversion constructors
	explicit Quaternion(const Vector3<float>& axis, float angle);
	explicit Quaternion(const Matrix3x3<float>& rotationMatrix);
	explicit Quaternion(const Matrix4x4<float>& rotationMatrix);
	
	// Euler angle constructors (angles in radians)
	static Quaternion FromEulerAngles(float pitch, float yaw, float roll);
	
	// Operators
	Quaternion operator*(const Quaternion& rhs) const;
	Quaternion& operator*=(const Quaternion& rhs);
	Vector3<float> operator*(const Vector3<float>& vec) const;
	
	bool operator==(const Quaternion& rhs) const;
	bool operator!=(const Quaternion& rhs) const;
	
	// Operations
	float Length() const;
	float LengthSquared() const;
	Quaternion Normalized() const;
	void Normalize();
	Quaternion Conjugate() const;
	Quaternion Inverse() const;
	
	// Conversions
	Vector3<float> ToEulerAngles() const;
	Matrix3x3<float> ToMatrix3() const;
	Matrix4x4<float> ToMatrix4() const;
	
	// Static operations
	static Quaternion Identity() { return Quaternion(0, 0, 0, 1); }
	static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t);
	static Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t);
	static float Dot(const Quaternion& a, const Quaternion& b);
};

// Common type aliases
using Vec2f = Vector2<float>;
using Vec2i = Vector2<int>;
using Vec3f = Vector3<float>;
using Vec3i = Vector3<int>;
using Vec4f = Vector4<float>;
using Vec4i = Vector4<int>;
using Mat3f = Matrix3x3<float>;
using Mat4f = Matrix4x4<float>;

} // namespace math
} // namespace glib

// Include inline implementations
#include "AlgebraMath.inl"