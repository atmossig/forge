#pragma once

#include "AlgebraMath.h"

namespace glib {
namespace math {

//------------------------------------------------------------------------------
// Vector2 implementations
//------------------------------------------------------------------------------

template <typename T>
Vector2<T> Vector2<T>::operator+(const Vector2<T>& rhs) const {
    return Vector2(x + rhs.x, y + rhs.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator-(const Vector2<T>& rhs) const {
    return Vector2(x - rhs.x, y - rhs.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator*(T scalar) const {
    return Vector2(x * scalar, y * scalar);
}

template <typename T>
Vector2<T> Vector2<T>::operator/(T scalar) const {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    return Vector2(x * invScalar, y * invScalar);
}

template <typename T>
Vector2<T>& Vector2<T>::operator+=(const Vector2<T>& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
}

template <typename T>
Vector2<T>& Vector2<T>::operator-=(const Vector2<T>& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

template <typename T>
Vector2<T>& Vector2<T>::operator*=(T scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

template <typename T>
Vector2<T>& Vector2<T>::operator/=(T scalar) {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    x *= invScalar;
    y *= invScalar;
    return *this;
}

template <typename T>
bool Vector2<T>::operator==(const Vector2<T>& rhs) const {
    return NearlyEqual(x, rhs.x) && NearlyEqual(y, rhs.y);
}

template <typename T>
bool Vector2<T>::operator!=(const Vector2<T>& rhs) const {
    return !(*this == rhs);
}

template <typename T>
T Vector2<T>::Length() const {
    return std::sqrt(x * x + y * y);
}

template <typename T>
T Vector2<T>::LengthSquared() const {
    return x * x + y * y;
}

template <typename T>
Vector2<T> Vector2<T>::Normalized() const {
    T len = Length();
    if (len < EPSILON)
        return Vector2<T>::Zero();
    T invLen = 1 / len;
    return Vector2(x * invLen, y * invLen);
}

template <typename T>
void Vector2<T>::Normalize() {
    T len = Length();
    if (len < EPSILON)
        return;
    T invLen = 1 / len;
    x *= invLen;
    y *= invLen;
}

template <typename T>
T Vector2<T>::Dot(const Vector2<T>& rhs) const {
    return x * rhs.x + y * rhs.y;
}

template <typename T>
T Vector2<T>::Cross(const Vector2<T>& rhs) const {
    return x * rhs.y - y * rhs.x;
}

template <typename T>
T Vector2<T>::Distance(const Vector2<T>& a, const Vector2<T>& b) {
    return (b - a).Length();
}

template <typename T>
T Vector2<T>::DistanceSquared(const Vector2<T>& a, const Vector2<T>& b) {
    return (b - a).LengthSquared();
}

template <typename T>
Vector2<T> Vector2<T>::Lerp(const Vector2<T>& a, const Vector2<T>& b, T t) {
    t = Clamp(t, 0, 1);
    return Vector2(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y)
    );
}

//------------------------------------------------------------------------------
// Vector3 implementations
//------------------------------------------------------------------------------

template <typename T>
Vector3<T> Vector3<T>::operator+(const Vector3<T>& rhs) const {
    return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator-(const Vector3<T>& rhs) const {
    return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator*(T scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
}

template <typename T>
Vector3<T> Vector3<T>::operator/(T scalar) const {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    return Vector3(x * invScalar, y * invScalar, z * invScalar);
}

template <typename T>
Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

template <typename T>
Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

template <typename T>
Vector3<T>& Vector3<T>::operator*=(T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

template <typename T>
Vector3<T>& Vector3<T>::operator/=(T scalar) {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    x *= invScalar;
    y *= invScalar;
    z *= invScalar;
    return *this;
}

template <typename T>
bool Vector3<T>::operator==(const Vector3<T>& rhs) const {
    return NearlyEqual(x, rhs.x) && NearlyEqual(y, rhs.y) && NearlyEqual(z, rhs.z);
}

template <typename T>
bool Vector3<T>::operator!=(const Vector3<T>& rhs) const {
    return !(*this == rhs);
}

template <typename T>
T Vector3<T>::Length() const {
    return std::sqrt(x * x + y * y + z * z);
}

template <typename T>
T Vector3<T>::LengthSquared() const {
    return x * x + y * y + z * z;
}

template <typename T>
Vector3<T> Vector3<T>::Normalized() const {
    T len = Length();
    if (len < EPSILON)
        return Vector3<T>::Zero();
    T invLen = 1 / len;
    return Vector3(x * invLen, y * invLen, z * invLen);
}

template <typename T>
void Vector3<T>::Normalize() {
    T len = Length();
    if (len < EPSILON)
        return;
    T invLen = 1 / len;
    x *= invLen;
    y *= invLen;
    z *= invLen;
}

template <typename T>
T Vector3<T>::Dot(const Vector3<T>& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

template <typename T>
Vector3<T> Vector3<T>::Cross(const Vector3<T>& rhs) const {
    return Vector3(
        y * rhs.z - z * rhs.y,
        z * rhs.x - x * rhs.z,
        x * rhs.y - y * rhs.x
    );
}

template <typename T>
T Vector3<T>::Distance(const Vector3<T>& a, const Vector3<T>& b) {
    return (b - a).Length();
}

template <typename T>
T Vector3<T>::DistanceSquared(const Vector3<T>& a, const Vector3<T>& b) {
    return (b - a).LengthSquared();
}

template <typename T>
Vector3<T> Vector3<T>::Lerp(const Vector3<T>& a, const Vector3<T>& b, T t) {
    t = Clamp(t, 0, 1);
    return Vector3(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z)
    );
}

template <typename T>
Vector3<T> Vector3<T>::Reflect(const Vector3<T>& direction, const Vector3<T>& normal) {
    T factor = -2.0f * direction.Dot(normal);
    return Vector3<T>(
        direction.x + factor * normal.x,
        direction.y + factor * normal.y,
        direction.z + factor * normal.z
    );
}

//------------------------------------------------------------------------------
// Vector4 implementations
//------------------------------------------------------------------------------

template <typename T>
Vector4<T> Vector4<T>::operator+(const Vector4<T>& rhs) const {
    return Vector4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

template <typename T>
Vector4<T> Vector4<T>::operator-(const Vector4<T>& rhs) const {
    return Vector4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
}

template <typename T>
Vector4<T> Vector4<T>::operator*(T scalar) const {
    return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
}

template <typename T>
Vector4<T> Vector4<T>::operator/(T scalar) const {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    return Vector4(x * invScalar, y * invScalar, z * invScalar, w * invScalar);
}

template <typename T>
Vector4<T>& Vector4<T>::operator+=(const Vector4<T>& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    w += rhs.w;
    return *this;
}

template <typename T>
Vector4<T>& Vector4<T>::operator-=(const Vector4<T>& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    w -= rhs.w;
    return *this;
}

template <typename T>
Vector4<T>& Vector4<T>::operator*=(T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    w *= scalar;
    return *this;
}

template <typename T>
Vector4<T>& Vector4<T>::operator/=(T scalar) {
    assert(scalar != 0);
    T invScalar = 1 / scalar;
    x *= invScalar;
    y *= invScalar;
    z *= invScalar;
    w *= invScalar;
    return *this;
}

template <typename T>
bool Vector4<T>::operator==(const Vector4<T>& rhs) const {
    return NearlyEqual(x, rhs.x) && NearlyEqual(y, rhs.y) && 
           NearlyEqual(z, rhs.z) && NearlyEqual(w, rhs.w);
}

template <typename T>
bool Vector4<T>::operator!=(const Vector4<T>& rhs) const {
    return !(*this == rhs);
}

template <typename T>
T Vector4<T>::Length() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

template <typename T>
T Vector4<T>::LengthSquared() const {
    return x * x + y * y + z * z + w * w;
}

template <typename T>
Vector4<T> Vector4<T>::Normalized() const {
    T len = Length();
    if (len < EPSILON)
        return Vector4<T>::Zero();
    T invLen = 1 / len;
    return Vector4(x * invLen, y * invLen, z * invLen, w * invLen);
}

template <typename T>
void Vector4<T>::Normalize() {
    T len = Length();
    if (len < EPSILON)
        return;
    T invLen = 1 / len;
    x *= invLen;
    y *= invLen;
    z *= invLen;
    w *= invLen;
}

template <typename T>
T Vector4<T>::Dot(const Vector4<T>& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
}

template <typename T>
T Vector4<T>::Distance(const Vector4<T>& a, const Vector4<T>& b) {
    return (b - a).Length();
}

template <typename T>
T Vector4<T>::DistanceSquared(const Vector4<T>& a, const Vector4<T>& b) {
    return (b - a).LengthSquared();
}

template <typename T>
Vector4<T> Vector4<T>::Lerp(const Vector4<T>& a, const Vector4<T>& b, T t) {
    t = Clamp(t, 0, 1);
    return Vector4(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z),
        a.w + t * (b.w - a.w)
    );
}

//------------------------------------------------------------------------------
// Matrix3x3 implementations
//------------------------------------------------------------------------------

template <typename T>
Matrix3x3<T>::Matrix3x3() {
    // Initialize to identity matrix
    m[0] = 1; m[1] = 0; m[2] = 0;
    m[3] = 0; m[4] = 1; m[5] = 0;
    m[6] = 0; m[7] = 0; m[8] = 1;
}

template <typename T>
Matrix3x3<T>::Matrix3x3(
    T m00, T m01, T m02,
    T m10, T m11, T m12,
    T m20, T m21, T m22
) {
    m[0] = m00; m[1] = m01; m[2] = m02;
    m[3] = m10; m[4] = m11; m[5] = m12;
    m[6] = m20; m[7] = m21; m[8] = m22;
}

template <typename T>
T& Matrix3x3<T>::operator()(int row, int col) {
    assert(row >= 0 && row < 3 && col >= 0 && col < 3);
    return m[row * 3 + col];
}

template <typename T>
T Matrix3x3<T>::operator()(int row, int col) const {
    assert(row >= 0 && row < 3 && col >= 0 && col < 3);
    return m[row * 3 + col];
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Identity() {
    return Matrix3x3();
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Translation(T x, T y) {
    return Matrix3x3(
        1, 0, x,
        0, 1, y,
        0, 0, 1
    );
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Translation(const Vector2<T>& translation) {
    return Translation(translation.x, translation.y);
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Rotation(T angleInRadians) {
    T c = std::cos(angleInRadians);
    T s = std::sin(angleInRadians);
    
    return Matrix3x3(
        c, -s, 0,
        s,  c, 0,
        0,  0, 1
    );
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Scaling(T x, T y) {
    return Matrix3x3(
        x, 0, 0,
        0, y, 0,
        0, 0, 1
    );
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Scaling(const Vector2<T>& scale) {
    return Scaling(scale.x, scale.y);
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::operator*(const Matrix3x3<T>& rhs) const {
    Matrix3x3<T> result;
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result(i, j) = 
                (*this)(i, 0) * rhs(0, j) +
                (*this)(i, 1) * rhs(1, j) +
                (*this)(i, 2) * rhs(2, j);
        }
    }
    
    return result;
}

template <typename T>
Vector3<T> Matrix3x3<T>::operator*(const Vector3<T>& vec) const {
    return Vector3<T>(
        m[0] * vec.x + m[1] * vec.y + m[2] * vec.z,
        m[3] * vec.x + m[4] * vec.y + m[5] * vec.z,
        m[6] * vec.x + m[7] * vec.y + m[8] * vec.z
    );
}

template <typename T>
Matrix3x3<T>& Matrix3x3<T>::operator*=(const Matrix3x3<T>& rhs) {
    *this = *this * rhs;
    return *this;
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Transposed() const {
    return Matrix3x3<T>(
        m[0], m[3], m[6],
        m[1], m[4], m[7],
        m[2], m[5], m[8]
    );
}

template <typename T>
T Matrix3x3<T>::Determinant() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

template <typename T>
Matrix3x3<T> Matrix3x3<T>::Inverse() const {
    T det = Determinant();
    assert(std::abs(det) > EPSILON);
    
    T invDet = 1 / det;
    
    Matrix3x3<T> result;
    result.m[0] = (m[4] * m[8] - m[5] * m[7]) * invDet;
    result.m[1] = (m[2] * m[7] - m[1] * m[8]) * invDet;
    result.m[2] = (m[1] * m[5] - m[2] * m[4]) * invDet;
    result.m[3] = (m[5] * m[6] - m[3] * m[8]) * invDet;
    result.m[4] = (m[0] * m[8] - m[2] * m[6]) * invDet;
    result.m[5] = (m[2] * m[3] - m[0] * m[5]) * invDet;
    result.m[6] = (m[3] * m[7] - m[4] * m[6]) * invDet;
    result.m[7] = (m[1] * m[6] - m[0] * m[7]) * invDet;
    result.m[8] = (m[0] * m[4] - m[1] * m[3]) * invDet;
    
    return result;
}

template <typename T>
bool Matrix3x3<T>::Invert() {
    T det = Determinant();
    if (std::abs(det) <= EPSILON)
        return false;
    
    *this = Inverse();
    return true;
}

//------------------------------------------------------------------------------
// Matrix4x4 implementations
//------------------------------------------------------------------------------

template <typename T>
Matrix4x4<T>::Matrix4x4() {
    // Initialize to identity matrix
    m[0] = 1; m[1] = 0; m[2] = 0; m[3] = 0;
    m[4] = 0; m[5] = 1; m[6] = 0; m[7] = 0;
    m[8] = 0; m[9] = 0; m[10] = 1; m[11] = 0;
    m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
}

template <typename T>
Matrix4x4<T>::Matrix4x4(
    T m00, T m01, T m02, T m03,
    T m10, T m11, T m12, T m13,
    T m20, T m21, T m22, T m23,
    T m30, T m31, T m32, T m33
) {
    m[0] = m00; m[1] = m01; m[2] = m02; m[3] = m03;
    m[4] = m10; m[5] = m11; m[6] = m12; m[7] = m13;
    m[8] = m20; m[9] = m21; m[10] = m22; m[11] = m23;
    m[12] = m30; m[13] = m31; m[14] = m32; m[15] = m33;
}

template <typename T>
T& Matrix4x4<T>::operator()(int row, int col) {
    assert(row >= 0 && row < 4 && col >= 0 && col < 4);
    return m[row * 4 + col];
}

template <typename T>
T Matrix4x4<T>::operator()(int row, int col) const {
    assert(row >= 0 && row < 4 && col >= 0 && col < 4);
    return m[row * 4 + col];
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Identity() {
    return Matrix4x4();
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Translation(T x, T y, T z) {
    return Matrix4x4(
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Translation(const Vector3<T>& translation) {
    return Translation(translation.x, translation.y, translation.z);
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::RotationX(T angleInRadians) {
    T c = std::cos(angleInRadians);
    T s = std::sin(angleInRadians);
    
    return Matrix4x4(
        1, 0,  0, 0,
        0, c, -s, 0,
        0, s,  c, 0,
        0, 0,  0, 1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::RotationY(T angleInRadians) {
    T c = std::cos(angleInRadians);
    T s = std::sin(angleInRadians);
    
    return Matrix4x4(
        c,  0, s, 0,
        0,  1, 0, 0,
       -s,  0, c, 0,
        0,  0, 0, 1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::RotationZ(T angleInRadians) {
    T c = std::cos(angleInRadians);
    T s = std::sin(angleInRadians);
    
    return Matrix4x4(
        c, -s, 0, 0,
        s,  c, 0, 0,
        0,  0, 1, 0,
        0,  0, 0, 1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Rotation(const Vector3<T>& axis, T angleInRadians) {
    Vector3<T> normalizedAxis = axis.Normalized();
    T c = std::cos(angleInRadians);
    T s = std::sin(angleInRadians);
    T t = 1 - c;
    
    T x = normalizedAxis.x;
    T y = normalizedAxis.y;
    T z = normalizedAxis.z;
    
    return Matrix4x4(
        t*x*x + c,   t*x*y - s*z, t*x*z + s*y, 0,
        t*x*y + s*z, t*y*y + c,   t*y*z - s*x, 0,
        t*x*z - s*y, t*y*z + s*x, t*z*z + c,   0,
        0,           0,           0,           1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Scaling(T x, T y, T z) {
    return Matrix4x4(
        x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1
    );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Scaling(const Vector3<T>& scale) {
    return Scaling(scale.x, scale.y, scale.z);
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Perspective(T fovY, T aspectRatio, T nearZ, T farZ) {
    assert(nearZ > 0 && farZ > nearZ);
    T tanHalfFovY = std::tan(fovY / 2);
    
    Matrix4x4 result;
    result.m[0] = 1 / (aspectRatio * tanHalfFovY);
    result.m[5] = 1 / tanHalfFovY;
    result.m[10] = -(farZ + nearZ) / (farZ - nearZ);
    result.m[11] = -(2 * farZ * nearZ) / (farZ - nearZ);
    result.m[14] = -1;
    result.m[15] = 0;
    
    return result;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Orthographic(T left, T right, T bottom, T top, T nearZ, T farZ) {
    Matrix4x4 result;
    
    result.m[0] = 2 / (right - left);
    result.m[5] = 2 / (top - bottom);
    result.m[10] = -2 / (farZ - nearZ);
    result.m[3] = -(right + left) / (right - left);
    result.m[7] = -(top + bottom) / (top - bottom);
    result.m[11] = -(farZ + nearZ) / (farZ - nearZ);
    
    return result;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::LookAt(const Vector3<T>& eye, const Vector3<T>& target, const Vector3<T>& up) {
    Vector3<T> f = (target - eye).Normalized();
    Vector3<T> r = f.Cross(up.Normalized()).Normalized();
    Vector3<T> u = r.Cross(f);
    
    Matrix4x4<T> result = Matrix4x4<T>::Identity();
    
    result.m[0] = r.x;
    result.m[1] = r.y;
    result.m[2] = r.z;
    result.m[3] = -r.Dot(eye);
    
    result.m[4] = u.x;
    result.m[5] = u.y;
    result.m[6] = u.z;
    result.m[7] = -u.Dot(eye);
    
    result.m[8] = -f.x;
    result.m[9] = -f.y;
    result.m[10] = -f.z;
    result.m[11] = f.Dot(eye);
    
    return result;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::operator*(const Matrix4x4<T>& rhs) const {
    Matrix4x4<T> result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result(i, j) = 
                (*this)(i, 0) * rhs(0, j) +
                (*this)(i, 1) * rhs(1, j) +
                (*this)(i, 2) * rhs(2, j) +
                (*this)(i, 3) * rhs(3, j);
        }
    }
    
    return result;
}

template <typename T>
Vector4<T> Matrix4x4<T>::operator*(const Vector4<T>& vec) const {
    return Vector4<T>(
        m[0] * vec.x + m[1] * vec.y + m[2] * vec.z + m[3] * vec.w,
        m[4] * vec.x + m[5] * vec.y + m[6] * vec.z + m[7] * vec.w,
        m[8] * vec.x + m[9] * vec.y + m[10] * vec.z + m[11] * vec.w,
        m[12] * vec.x + m[13] * vec.y + m[14] * vec.z + m[15] * vec.w
    );
}

template <typename T>
Matrix4x4<T>& Matrix4x4<T>::operator*=(const Matrix4x4<T>& rhs) {
    *this = *this * rhs;
    return *this;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Transposed() const {
    return Matrix4x4<T>(
        m[0], m[4], m[8],  m[12],
        m[1], m[5], m[9],  m[13],
        m[2], m[6], m[10], m[14],
        m[3], m[7], m[11], m[15]
    );
}

template <typename T>
T Matrix4x4<T>::Determinant() const {
    // Cofactor expansion along the first row
    return m[0] * (
               m[5] * (m[10] * m[15] - m[11] * m[14]) -
               m[9] * (m[6] * m[15] - m[7] * m[14]) +
               m[13] * (m[6] * m[11] - m[7] * m[10])
           ) - 
           m[1] * (
               m[4] * (m[10] * m[15] - m[11] * m[14]) -
               m[8] * (m[6] * m[15] - m[7] * m[14]) +
               m[12] * (m[6] * m[11] - m[7] * m[10])
           ) +
           m[2] * (
               m[4] * (m[9] * m[15] - m[11] * m[13]) -
               m[8] * (m[5] * m[15] - m[7] * m[13]) +
               m[12] * (m[5] * m[11] - m[7] * m[9])
           ) -
           m[3] * (
               m[4] * (m[9] * m[14] - m[10] * m[13]) -
               m[8] * (m[5] * m[14] - m[6] * m[13]) +
               m[12] * (m[5] * m[10] - m[6] * m[9])
           );
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::Inverse() const {
    // Calculate the inverse using cofactors
    T coef00 = m[10] * m[15] - m[11] * m[14];
    T coef02 = m[6] * m[15] - m[7] * m[14];
    T coef03 = m[6] * m[11] - m[7] * m[10];
    
    T coef04 = m[9] * m[15] - m[11] * m[13];
    T coef06 = m[5] * m[15] - m[7] * m[13];
    T coef07 = m[5] * m[11] - m[7] * m[9];
    
    T coef08 = m[9] * m[14] - m[10] * m[13];
    T coef10 = m[5] * m[14] - m[6] * m[13];
    T coef11 = m[5] * m[10] - m[6] * m[9];
    
    T coef12 = m[8] * m[15] - m[11] * m[12];
    T coef14 = m[4] * m[15] - m[7] * m[12];
    T coef15 = m[4] * m[11] - m[7] * m[8];
    
    T coef16 = m[8] * m[14] - m[10] * m[12];
    T coef18 = m[4] * m[14] - m[6] * m[12];
    T coef19 = m[4] * m[10] - m[6] * m[8];
    
    T coef20 = m[8] * m[13] - m[9] * m[12];
    T coef22 = m[4] * m[13] - m[5] * m[12];
    T coef23 = m[4] * m[9] - m[5] * m[8];
    
    Vector4<T> fac0(coef00, coef00, coef02, coef03);
    Vector4<T> fac1(coef04, coef04, coef06, coef07);
    Vector4<T> fac2(coef08, coef08, coef10, coef11);
    Vector4<T> fac3(coef12, coef12, coef14, coef15);
    Vector4<T> fac4(coef16, coef16, coef18, coef19);
    Vector4<T> fac5(coef20, coef20, coef22, coef23);
    
    Vector4<T> vec0(m[1], m[0], m[0], m[0]);
    Vector4<T> vec1(m[5], m[4], m[4], m[4]);
    Vector4<T> vec2(m[9], m[8], m[8], m[8]);
    Vector4<T> vec3(m[13], m[12], m[12], m[12]);
    
    Vector4<T> inv0(vec1 * fac0 - vec2 * fac1 + vec3 * fac2);
    Vector4<T> inv1(vec0 * fac0 - vec2 * fac3 + vec3 * fac4);
    Vector4<T> inv2(vec0 * fac1 - vec1 * fac3 + vec3 * fac5);
    Vector4<T> inv3(vec0 * fac2 - vec1 * fac4 + vec2 * fac5);
    
    Vector4<T> signA(1, -1, 1, -1);
    Vector4<T> signB(-1, 1, -1, 1);
    
    Matrix4x4<T> inverse;
    inverse.m[0] = inv0.x * signA.x;
    inverse.m[1] = inv0.y * signA.y;
    inverse.m[2] = inv0.z * signA.z;
    inverse.m[3] = inv0.w * signA.w;
    
    inverse.m[4] = inv1.x * signB.x;
    inverse.m[5] = inv1.y * signB.y;
    inverse.m[6] = inv1.z * signB.z;
    inverse.m[7] = inv1.w * signB.w;
    
    inverse.m[8] = inv2.x * signA.x;
    inverse.m[9] = inv2.y * signA.y;
    inverse.m[10] = inv2.z * signA.z;
    inverse.m[11] = inv2.w * signA.w;
    
    inverse.m[12] = inv3.x * signB.x;
    inverse.m[13] = inv3.y * signB.y;
    inverse.m[14] = inv3.z * signB.z;
    inverse.m[15] = inv3.w * signB.w;
    
    // Calculate determinant
    T determinant = 
        m[0] * inverse.m[0] +
        m[1] * inverse.m[4] +
        m[2] * inverse.m[8] +
        m[3] * inverse.m[12];
    
    assert(std::abs(determinant) > EPSILON);
    
    // Multiply by reciprocal of determinant
    T invDet = 1 / determinant;
    for (int i = 0; i < 16; ++i) {
        inverse.m[i] *= invDet;
    }
    
    return inverse;
}

template <typename T>
bool Matrix4x4<T>::Invert() {
    T det = Determinant();
    if (std::abs(det) <= EPSILON)
        return false;
    
    *this = Inverse();
    return true;
}

} // namespace math
} // namespace glib