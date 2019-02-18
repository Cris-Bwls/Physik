#include "Poly.h"
#include <Gizmos.h>

Poly::Poly(vector<vec2> const & vertices, vec2 position, vec2 velocity, float rotation, float mass, float elasticity, glm::vec4 colour) :
	RigidBody::RigidBody(ShapeID::Poly, position, velocity, rotation, mass, elasticity)
{
	m_Colour = colour;
	m_Vertices = vertices;
}

Poly::~Poly()
{
}

void Poly::CreateBroadColl()
{
	float radius = 0;
	for each (vec2 vert in m_Vertices)
	{
		float temp = length(vert);
		if (radius < temp)
			radius = temp;
	}
	radius += 0.1f;
	
	if (m_pBroadColl)
		delete m_pBroadColl;

	m_pBroadColl = new Sphere(m_position, m_velocity, m_mass, m_elasticity, radius, m_Colour);
}
