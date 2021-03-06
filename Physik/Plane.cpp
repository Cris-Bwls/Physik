#include "Plane.h"
#include <Gizmos.h>

using aie::Gizmos;

Plane::Plane(vec2 normal, float distance, float fFricCoStatic, float fFricCoDynamic) : PhysicsObject::PhysicsObject(ShapeID::Plane, fFricCoStatic, fFricCoDynamic)
{
	m_normal = normalize(normal);
	m_distanceToOrigin = distance;
}


Plane::~Plane()
{
}

void Plane::makeGizmo()
{
	float lineSegmentLength = 300;
	vec2 centerPoint = m_normal * m_distanceToOrigin;
	vec2 parallel(m_normal.y, -m_normal.x);
	vec4 colour(1, 1, 1, 1);
	vec2 start = centerPoint + (parallel * lineSegmentLength);
	vec2 end = centerPoint - (parallel * lineSegmentLength);

	Gizmos::add2DLine(start, end, colour);
}

void Plane::resetPosition()
{
}

void Plane::resolveCollision(RigidBody * actor2, vec2 const & normal)
{

	float j = dot(-(1 + actor2->getElasticity()) * actor2->getVelocity(), normal) /
		dot(normal, normal * (1 / actor2->getMass()));

	vec2 force = normal * j;

	actor2->applyForce(force);
}
