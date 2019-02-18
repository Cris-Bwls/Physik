#include "Sphere.h"
#include <Gizmos.h>

using aie::Gizmos;

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

	glm::vec2 startPoint = m_position + (glm::normalize(m_velocity) * m_radius);
	DebugVelocity(startPoint);
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
