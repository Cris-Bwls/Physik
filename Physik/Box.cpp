#include "Box.h"
#include <Gizmos.h>


Box::Box(glm::vec2 extents, glm::vec2 position, glm::vec2 velocity, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag, glm::vec4 colour, bool bIsFilled) :
	RigidBody::RigidBody(ShapeID::Box, position, velocity, 0, 0, mass, elasticity, fFricCoStatic, fFricCoDynamic, fDrag, fAngDrag)
{
	m_Extents = extents;

	m_Colour = colour;
	m_bIsFilled = bIsFilled;
}


Box::~Box()
{
}

void Box::makeGizmo()
{
	if (m_bIsFilled)
		aie::Gizmos::add2DAABBFilled(m_position, m_Extents, m_Colour);
	else
		aie::Gizmos::add2DAABB(m_position, m_Extents, m_Colour);
}
