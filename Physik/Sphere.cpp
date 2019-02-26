#include "Sphere.h"
#include <Gizmos.h>

using aie::Gizmos;
using namespace glm;

Sphere::Sphere(glm::vec2 position, glm::vec2 velocity, float mass, float elasticity, float radius, glm::vec4 colour) :
	RigidBody::RigidBody(ShapeID::Sphere, position, velocity, 0, mass, elasticity)
{
	m_radius = radius;
	m_colour = colour;
}

Sphere::~Sphere()
{
}

void Sphere::makeGizmo()
{
	Gizmos::add2DCircle(m_position, m_radius, 69U, m_colour);

	vec2 startPoint = m_position + (normalize(m_velocity) * m_radius);
	DebugVelocity(startPoint);

	if (m_bDirLine)
	{
		mat2 rotMat;
		rotMat[0][0] = cosf(m_rotation);
		rotMat[0][1] = sinf(m_rotation);
		rotMat[1][0] = -sinf(m_rotation);
		rotMat[1][1] = cosf(m_rotation);

		vec2 result = rotMat * vec2(0, m_radius);

		vec4 invertColor = { 1,1,1,1 };
		invertColor -= m_colour;
		invertColor.a = 1;

		Gizmos::add2DLine(m_position, m_position + result, invertColor);
	}
}

bool Sphere::checkCollision(PhysicsObject * pOther)
{
	Sphere* pOtherSphere = dynamic_cast<Sphere*>(pOther);

	// IF successful cast
	if (pOtherSphere)
	{
		float seperation = glm::distance(this->getPosition(), pOtherSphere->getPosition());
		float fRadiusSum = this->getRadius() + pOtherSphere->getRadius();

		// IF collision
		if (seperation < fRadiusSum)
			return true;
		else
			return false;
	}
	
	return false;
}
