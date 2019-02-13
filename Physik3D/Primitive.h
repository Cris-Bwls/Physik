#pragma once
#include <glm/ext.hpp>

using namespace glm;

enum class PrimitiveID
{
	NONE = 0,
	Plane,
	Sphere,
	AABB,
	Box,
	
	TOTAL
};

class Primitive
{
public:
	inline PrimitiveID GetID() { return m_ID; };
protected:
	inline Primitive(PrimitiveID id) { m_ID = id; };
	virtual ~Primitive() = 0;

	PrimitiveID m_ID;
};