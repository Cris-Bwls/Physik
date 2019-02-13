#include "Plane.h"
#include <Gizmos.h>

using aie::Gizmos;

Plane::Plane() : PhysicsObject::PhysicsObject(ShapeID::Plane)
{
	m_normal = { 0, 1 };
	m_distanceToOrigin = 0;
}

Plane::Plane(vec2 normal, float distance) : PhysicsObject::PhysicsObject(ShapeID::Plane)
{
	m_normal = normal;
	m_distanceToOrigin = distance;
}


Plane::~Plane()
{
}

void Plane::fixedUpdate(vec2 gravity, float timeStep)
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