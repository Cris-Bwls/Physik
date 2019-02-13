#pragma once
#include "Primitive.h"

class AABB : Primitive
{
public:
	AABB(vec3 const& extents, vec3 const& pos) : Primitive(PrimitiveID::AABB)
	{
		m_extents = extents;
		m_pos = pos;
	}

	inline vec3 GetExtents() { return m_extents; };
	inline vec3 GetPos() { return m_pos; };

private:
	vec3 m_extents;
	vec3 m_pos;
};