//////////////////////////////////////////////////////////////////////////////////////////
//	VECTOR3D.h
//	Class declaration for a 3d vector
//	Downloaded from: www.paulsprojects.net
//	Modified to include utility functions for cross-product, normalization, and `LoadZero()`
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <cmath>

class VECTOR3D
{
public:
	// Constructors
	VECTOR3D(void) : x(0.0f), y(0.0f), z(0.0f) {}

	VECTOR3D(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {}

	VECTOR3D(const float* rhs) : x(*rhs), y(*(rhs + 1)), z(*(rhs + 2)) {}

	VECTOR3D(const VECTOR3D& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}

	~VECTOR3D() {}	// Empty destructor

	void Set(float newX, float newY, float newZ)
	{
		x = newX; y = newY; z = newZ;
	}

	// Utility functions for vector algebra
	VECTOR3D CrossProduct(const VECTOR3D& rhs) const
	{
		return VECTOR3D(
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x
		);
	}

	float DotProduct(const VECTOR3D& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	VECTOR3D Normalized() const
	{
		float norm = GetLength();
		return (norm > 0.0f) ? VECTOR3D(x / norm, y / norm, z / norm) : VECTOR3D(0.0f, 0.0f, 0.0f);
	}

	void Normalize()
	{
		float norm = GetLength();
		if (norm > 0.0f)
		{
			x /= norm; y /= norm; z /= norm;
		}
	}

	float GetLength() const
	{
		return std::sqrt(x * x + y * y + z * z);
	}

	// Reintroduced `LoadZero` for compatibility
	void LoadZero(void)
	{
		x = 0.0f; y = 0.0f; z = 0.0f;
	}

	// Overloaded operators for vector operations
	VECTOR3D operator+(const VECTOR3D& rhs) const
	{
		return VECTOR3D(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	VECTOR3D operator-(const VECTOR3D& rhs) const
	{
		return VECTOR3D(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	VECTOR3D operator*(float scalar) const
	{
		return VECTOR3D(x * scalar, y * scalar, z * scalar);
	}

	VECTOR3D operator/(float scalar) const
	{
		return (scalar != 0.0f) ? VECTOR3D(x / scalar, y / scalar, z / scalar) : VECTOR3D(0.0f, 0.0f, 0.0f);
	}

	// Unary minus operator
	VECTOR3D operator-(void) const { return VECTOR3D(-x, -y, -z); }

	// Self-assignment operators
	void operator+=(const VECTOR3D& rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
	}

	void operator-=(const VECTOR3D& rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
	}

	void operator*=(float scalar)
	{
		x *= scalar; y *= scalar; z *= scalar;
	}

	void operator/=(float scalar)
	{
		if (scalar != 0.0f)
		{
			x /= scalar; y /= scalar; z /= scalar;
		}
	}

	// Member variables
	float x, y, z;
};

// Non-member utility functions
inline VECTOR3D cross(const VECTOR3D& v1, const VECTOR3D& v2)
{
	return v1.CrossProduct(v2);
}

inline VECTOR3D normalize(const VECTOR3D& v)
{
	return v.Normalized();
}

#endif // VECTOR3D_H
