#pragma once
#include "Primitive.h"

class Sphere : Primitive
{
public:
	Sphere(vec3 const& center, float const& radius) : Primitive(PrimitiveID::Sphere)
	{
		m_center = center;
		m_radius = radius;
	}

protected:
	vec3 m_center;
	float m_radius;
};