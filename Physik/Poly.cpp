#include "Poly.h"
#include <Gizmos.h>

#define DEBUG true

Poly::Poly(vector<vec2> const & vertices, vec2 position, vec2 velocity, float rotation, float mass, float elasticity, glm::vec4 colour) :
	RigidBody::RigidBody(ShapeID::Poly, position, velocity, rotation, mass, elasticity)
{
	m_Colour = colour;
	m_Vertices = vertices;
}

Poly::~Poly()
{
}

void Poly::makeGizmo()
{
	vec2 start;
	vec2 end;
	uint count = m_Vertices.size();
	for (int i = 0; i < count; ++i)
	{
		start = m_Vertices[i] + m_position;
		if (i + 1 < count)
			end = m_Vertices[i + 1] + m_position;
		else
			end = m_Vertices[0] + m_position;

		aie::Gizmos::add2DTri(start, end, m_position, m_Colour);
	}

	if (DEBUG)
		m_pBroadColl->makeGizmo();
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
