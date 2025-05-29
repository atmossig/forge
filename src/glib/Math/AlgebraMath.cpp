#include "AlgebraMath.h"

namespace glib {
namespace math {

// Quaternion from axis and angle
Quaternion::Quaternion(const Vector3<float>& axis, float angle) {
	float halfAngle = angle * 0.5f;
	float sinHalfAngle = std::sin(halfAngle);
	
	// Ensure the axis is normalized
	Vector3<float> normalizedAxis = axis.Normalized();
	x = normalizedAxis.x * sinHalfAngle;
	y = normalizedAxis.y * sinHalfAngle;
	z = normalizedAxis.z * sinHalfAngle;
	w = std::cos(halfAngle);
}

// Quaternion from 3x3 rotation matrix
Quaternion::Quaternion(const Matrix3x3<float>& rotMat) {
	float trace = rotMat(0, 0) + rotMat(1, 1) + rotMat(2, 2);
	
	if (trace > 0.0f) {
		float s = 0.5f / std::sqrt(trace + 1.0f);
		w = 0.25f / s;
		x = (rotMat(2, 1) - rotMat(1, 2)) * s;
		y = (rotMat(0, 2) - rotMat(2, 0)) * s;
		z = (rotMat(1, 0) - rotMat(0, 1)) * s;
	} else {
		if (rotMat(0, 0) > rotMat(1, 1) && rotMat(0, 0) > rotMat(2, 2)) {
			float s = 2.0f * std::sqrt(1.0f + rotMat(0, 0) - rotMat(1, 1) - rotMat(2, 2));
			w = (rotMat(2, 1) - rotMat(1, 2)) / s;
			x = 0.25f * s;
			y = (rotMat(0, 1) + rotMat(1, 0)) / s;
			z = (rotMat(0, 2) + rotMat(2, 0)) / s;
		} else if (rotMat(1, 1) > rotMat(2, 2)) {
			float s = 2.0f * std::sqrt(1.0f + rotMat(1, 1) - rotMat(0, 0) - rotMat(2, 2));
			w = (rotMat(0, 2) - rotMat(2, 0)) / s;
			x = (rotMat(0, 1) + rotMat(1, 0)) / s;
			y = 0.25f * s;
			z = (rotMat(1, 2) + rotMat(2, 1)) / s;
		} else {
			float s = 2.0f * std::sqrt(1.0f + rotMat(2, 2) - rotMat(0, 0) - rotMat(1, 1));
			w = (rotMat(1, 0) - rotMat(0, 1)) / s;
			x = (rotMat(0, 2) + rotMat(2, 0)) / s;
			y = (rotMat(1, 2) + rotMat(2, 1)) / s;
			z = 0.25f * s;
		}
	}
	Normalize();
}

// Quaternion from 4x4 rotation matrix (uses the upper 3x3 portion)
Quaternion::Quaternion(const Matrix4x4<float>& rotMat) {
	Matrix3x3<float> mat3(
		rotMat(0, 0), rotMat(0, 1), rotMat(0, 2),
		rotMat(1, 0), rotMat(1, 1), rotMat(1, 2),
		rotMat(2, 0), rotMat(2, 1), rotMat(2, 2)
	);
	*this = Quaternion(mat3);
}

// Create quaternion from Euler angles (pitch, yaw, roll)
Quaternion Quaternion::FromEulerAngles(float pitch, float yaw, float roll) {
	// Convert to radians if needed
	float halfPitch = pitch * 0.5f;
	float halfYaw = yaw * 0.5f;
	float halfRoll = roll * 0.5f;
	
	float cosYaw = std::cos(halfYaw);
	float sinYaw = std::sin(halfYaw);
	float cosPitch = std::cos(halfPitch);
	float sinPitch = std::sin(halfPitch);
	float cosRoll = std::cos(halfRoll);
	float sinRoll = std::sin(halfRoll);
	
	Quaternion q;
	q.x = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	q.y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	q.z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	
	return q;
}

// Quaternion multiplication
Quaternion Quaternion::operator*(const Quaternion& rhs) const {
	return Quaternion(
		w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
		w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
		w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
		w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z
	);
}

Quaternion& Quaternion::operator*=(const Quaternion& rhs) {
	*this = *this * rhs;
	return *this;
}

// Rotate a vector by this quaternion
Vector3<float> Quaternion::operator*(const Vector3<float>& vec) const {
	// Convert vector to quaternion (with w=0)
	Quaternion vecQ(vec.x, vec.y, vec.z, 0.0f);
	
	// q * v * q^-1 (quaternion-vector rotation formula)
	Quaternion result = *this * vecQ * this->Conjugate();
	return Vector3<float>(result.x, result.y, result.z);
}

bool Quaternion::operator==(const Quaternion& rhs) const {
	return NearlyEqual(x, rhs.x) &&
		   NearlyEqual(y, rhs.y) &&
		   NearlyEqual(z, rhs.z) &&
		   NearlyEqual(w, rhs.w);
}

bool Quaternion::operator!=(const Quaternion& rhs) const {
	return !(*this == rhs);
}

float Quaternion::Length() const {
	return std::sqrt(x * x + y * y + z * z + w * w);
}

float Quaternion::LengthSquared() const {
	return x * x + y * y + z * z + w * w;
}

Quaternion Quaternion::Normalized() const {
	float len = Length();
	if (len < EPSILON) {
		return Quaternion::Identity();
	}
	
	float invLength = 1.0f / len;
	return Quaternion(
		x * invLength,
		y * invLength,
		z * invLength,
		w * invLength
	);
}

void Quaternion::Normalize() {
	float len = Length();
	if (len < EPSILON) {
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		w = 1.0f;
		return;
	}
	
	float invLength = 1.0f / len;
	x *= invLength;
	y *= invLength;
	z *= invLength;
	w *= invLength;
}

Quaternion Quaternion::Conjugate() const {
	return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::Inverse() const {
	float lengthSq = LengthSquared();
	if (lengthSq < EPSILON) {
		return Quaternion::Identity();
	}
	
	// For unit quaternions, inverse equals conjugate
	// For non-unit quaternions, we need to divide by square of length
	float invLengthSq = 1.0f / lengthSq;
	return Quaternion(
		-x * invLengthSq,
		-y * invLengthSq,
		-z * invLengthSq,
		 w * invLengthSq
	);
}

Vector3<float> Quaternion::ToEulerAngles() const {
	Vector3<float> angles;
	
	// Pitch (x-axis rotation)
	float sinp = 2.0f * (w * x - y * z);
	if (std::abs(sinp) >= 1.0f) {
		angles.x = std::copysign(HALF_PI, sinp); // Use 90 degrees if out of range
	} else {
		angles.x = std::asin(sinp);
	}
	
	// Yaw (y-axis rotation)
	float siny_cosp = 2.0f * (w * y + z * x);
	float cosy_cosp = 1.0f - 2.0f * (x * x + y * y);
	angles.y = std::atan2(siny_cosp, cosy_cosp);
	
	// Roll (z-axis rotation)
	float sinr_cosp = 2.0f * (w * z + x * y);
	float cosr_cosp = 1.0f - 2.0f * (y * y + z * z);
	angles.z = std::atan2(sinr_cosp, cosr_cosp);
	
	return angles;
}

Matrix3x3<float> Quaternion::ToMatrix3() const {
	Quaternion q = this->Normalized();
	float xx = q.x * q.x;
	float xy = q.x * q.y;
	float xz = q.x * q.z;
	float xw = q.x * q.w;
	float yy = q.y * q.y;
	float yz = q.y * q.z;
	float yw = q.y * q.w;
	float zz = q.z * q.z;
	float zw = q.z * q.w;
	
	return Matrix3x3<float>(
		1.0f - 2.0f * (yy + zz),  2.0f * (xy - zw),        2.0f * (xz + yw),
		2.0f * (xy + zw),         1.0f - 2.0f * (xx + zz), 2.0f * (yz - xw),
		2.0f * (xz - yw),         2.0f * (yz + xw),        1.0f - 2.0f * (xx + yy)
	);
}

Matrix4x4<float> Quaternion::ToMatrix4() const {
	Matrix3x3<float> mat3 = ToMatrix3();
	
	return Matrix4x4<float>(
		mat3(0,0), mat3(0,1), mat3(0,2), 0.0f,
		mat3(1,0), mat3(1,1), mat3(1,2), 0.0f,
		mat3(2,0), mat3(2,1), mat3(2,2), 0.0f,
		0.0f,      0.0f,      0.0f,      1.0f
	);
}

// Spherical linear interpolation between quaternions
Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t) {
	// Clamp t to range [0, 1]
	t = Clamp(t, 0.0f, 1.0f);
	
	Quaternion qa = a.Normalized();
	Quaternion qb = b.Normalized();
	
	// Calculate cosine of angle between quaternions
	float cosHalfTheta = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
	
	// If qa=qb or qa=-qb, we can return qa
	if (std::abs(cosHalfTheta) >= 1.0f) {
		return qa;
	}
	
	// If the dot product is negative, slerp won't take the shorter path
	// Fix by inverting one quaternion
	if (cosHalfTheta < 0.0f) {
		qb.x = -qb.x; qb.y = -qb.y; qb.z = -qb.z; qb.w = -qb.w;
		cosHalfTheta = -cosHalfTheta;
	}
	
	// Calculate interpolation factors
	float halfTheta = std::acos(cosHalfTheta);
	float sinHalfTheta = std::sqrt(1.0f - cosHalfTheta * cosHalfTheta);
	
	// If theta = 180 degrees, rotation not defined
	if (std::abs(sinHalfTheta) < EPSILON) {
		return Quaternion(
			qa.x * 0.5f + qb.x * 0.5f,
			qa.y * 0.5f + qb.y * 0.5f,
			qa.z * 0.5f + qb.z * 0.5f,
			qa.w * 0.5f + qb.w * 0.5f
		);
	}
	
	float ratioA = std::sin((1.0f - t) * halfTheta) / sinHalfTheta;
	float ratioB = std::sin(t * halfTheta) / sinHalfTheta;
	
	return Quaternion(
		qa.x * ratioA + qb.x * ratioB,
		qa.y * ratioA + qb.y * ratioB,
		qa.z * ratioA + qb.z * ratioB,
		qa.w * ratioA + qb.w * ratioB
	);
}

// Linear interpolation between quaternions (less accurate but faster than slerp)
Quaternion Quaternion::Lerp(const Quaternion& a, const Quaternion& b, float t) {
	t = Clamp(t, 0.0f, 1.0f);
	
	Quaternion result;
	float dot = Dot(a, b);
	
	if (dot < 0.0f) {
		result.x = a.x + t * (-b.x - a.x);
		result.y = a.y + t * (-b.y - a.y);
		result.z = a.z + t * (-b.z - a.z);
		result.w = a.w + t * (-b.w - a.w);
	} else {
		result.x = a.x + t * (b.x - a.x);
		result.y = a.y + t * (b.y - a.y);
		result.z = a.z + t * (b.z - a.z);
		result.w = a.w + t * (b.w - a.w);
	}
	
	result.Normalize();
	return result;
}

float Quaternion::Dot(const Quaternion& a, const Quaternion& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

} // namespace math
} // namespace glib