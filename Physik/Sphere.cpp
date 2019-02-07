#include "Sphere.h"
#include <Gizmos.h>

using aie::Gizmos;

Sphere::Sphere(glm::vec2 position, glm::vec2 velocity, float mass, float radius, glm::vec4 colour) :
	Rigidbody::Rigidbody(ShapeID::Sphere, position, velocity, 0, mass)
{
	m_radius = radius;
	m_colour = colour;
}

Sphere::~Sphere()
{
}

void Sphere::makeGizmo()
{
	Gizmos::add2DCircle();
}

bool Sphere::checkCollision(PhysicsObject * pOther)
{
	return false;
}
