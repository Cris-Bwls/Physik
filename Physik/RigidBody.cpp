#include "RigidBody.h"
#include <iostream>

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

void Rigidbody::fixedUpdate(glm::vec2 gravity, float timeStep)
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

void Rigidbody::applyForce(glm::vec2 force)
{
	m_velocity += force / m_mass;
}

void Rigidbody::applyForceToActor(Rigidbody* actor2, glm::vec2 force)
{
	applyForce(force);
	actor2->applyForce(-force);
}
