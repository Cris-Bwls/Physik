#pragma once
#include "Primitive.h"

class Plane : Primitive
{
public:
	Plane(vec3 const& normal, float const& distance) : Primitive::Primitive(PrimitiveID::Plane)
	{
		m_normal = normalize(normal);
		m_distance = distance;
	}

	static Plane ComputePlane(vec3 const& a, vec3 const& b, vec3 const& c)
	{
		vec3 normal = normalize(cross(b - a, c - a));
		float distance = dot(normal, a);

		return Plane(normal, distance);
	}

	inline vec3 GetNormal() const { return m_normal; };
	inline float GetDistance() const { return m_distance; };

protected:
	vec3 m_normal;
	float m_distance;
};