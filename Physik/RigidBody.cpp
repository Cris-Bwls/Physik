#include "RigidBody.h"
#include <iostream>

using namespace glm;

Rigidbody::Rigidbody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float mass, float elasticity) : PhysicsObject::PhysicsObject(shapeID)
{
	m_position = position; 
	m_velocity = velocity;
	m_rotation = rotation;
	m_mass = mass;
	m_elasticity = elasticity;
}

Rigidbody::~Rigidbody()
{
}

void Rigidbody::fixedUpdate(vec2 const& gravity, float timeStep)
{
	applyForce(gravity * m_mass * timeStep);
	m_position += m_velocity * timeStep;
}

void Rigidbody::debug()
{
	printf(" ID %i ", (int)m_ShapeId);
	printf(" POS x %f, y %f ", m_position.x, m_position.y);
	printf(" VEL x %f, y %f ", m_velocity.x, m_velocity.y);
	printf("\n");

}

void Rigidbody::applyForce(vec2 const& force)
{
	m_velocity += force / m_mass;
}

void Rigidbody::applyForceToActor(Rigidbody* actor2, vec2 const& force)
{
	applyForce(force);
	actor2->applyForce(-force);
}

void Rigidbody::resolveCollision(Rigidbody* actor2, vec2 const& normal)
{
	vec2 relativeVelocity = actor2->getVelocity() - m_velocity;
	float elasticity = (actor2->getElasticity() + m_elasticity) / 2;

	float j = dot(-(1 + elasticity) * relativeVelocity, normal) /
				dot(normal, normal * ((1 / m_mass + (1 / actor2->getMass()))));

	vec2 force = normal * j;

	applyForceToActor(actor2, -force);
}
