#include "RigidBody.h"
#include <iostream>
#include <Gizmos.h>

using namespace glm;

RigidBody::RigidBody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float mass, float elasticity) : PhysicsObject::PhysicsObject(shapeID)
{
	m_position = position;
	m_prevPosition = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_mass = mass;
	m_elasticity = elasticity;
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

	m_prevPosition = m_position;

	applyForce(gravity * m_mass * timeStep);
	m_position += m_velocity * timeStep;
}

void RigidBody::debug()
{
	printf(" ID %i ", (int)m_ShapeId);
	printf(" POS x %f, y %f ", m_position.x, m_position.y);
	printf(" VEL x %f, y %f ", m_velocity.x, m_velocity.y);	
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
