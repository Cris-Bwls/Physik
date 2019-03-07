#include "RigidBody.h"
#include <iostream>
#include <Gizmos.h>

using namespace glm;

RigidBody::RigidBody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float fAngVelocity,  float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag) 
	: PhysicsObject::PhysicsObject(shapeID, fFricCoStatic, fFricCoDynamic)
{
	m_position = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_angularVelocity = fAngVelocity;
	m_mass = mass;
	m_elasticity = elasticity;
	m_drag = fDrag;
	m_angularDrag = fAngDrag;

	m_ResolutionForceSum = { 0,0 };
}

RigidBody::~RigidBody()
{
}

void RigidBody::fixedUpdate(vec2 const& gravity, float timeStep)
{
	if (m_position.x != m_position.x)
	{
		printf(" FUCK");
	}

	applyForce(gravity * m_mass * timeStep);
	ApplyDrags(timeStep);

	m_position += m_velocity * timeStep;
	m_rotation += m_angularVelocity * timeStep;
}

void RigidBody::debug()
{
	printf(" ID %i ", (int)m_ShapeId);
	printf(" POS x %f, y %f ", m_position.x, m_position.y);
	printf(" VEL x %f, y %f ", m_velocity.x, m_velocity.y);	
	printf(" ROT %f ", m_rotation);
	printf(" ANG VEL %f ", m_angularVelocity);
}

void RigidBody::applyForce(vec2 const& force)
{
	m_velocity += force / m_mass;
}

void RigidBody::applyForceToActor(RigidBody* actor2, vec2 const& force)
{
	applyForce(force);
	actor2->applyForce(-force);
}

void RigidBody::ApplyDrags(float const& timeStep)
{
	m_velocity -= m_velocity * m_drag * timeStep;
	m_angularVelocity -= m_angularVelocity * m_angularDrag * timeStep;
}

void RigidBody::DebugVelocity(vec2 const& startPoint)
{
	vec2 endPoint = startPoint + (m_velocity * 0.5f);
	aie::Gizmos::add2DLine(startPoint, endPoint, { 1,1,1,1 });
}

void RigidBody::resolveCollision(RigidBody* actor2, vec2 const& normal)
{
	vec2 relativeVelocity = actor2->getVelocity() - m_velocity;
	float elasticity = (actor2->getElasticity() + m_elasticity) / 2;

	float j = dot(-(1 + elasticity) * relativeVelocity, normal) /
				dot(normal, normal * ((1 / m_mass + (1 / actor2->getMass()))));

	vec2 force = normal * j;
	if (force.x != force.x)
		printf("FUCK");

	applyForceToActor(actor2, -force);
}
