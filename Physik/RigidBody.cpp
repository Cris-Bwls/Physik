#include "RigidBody.h"
#include <iostream>

Rigidbody::Rigidbody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float mass) : PhysicsObject::PhysicsObject(shapeID)
{
	m_position = position; 
	m_velocity = velocity;
	m_rotation = rotation;
	m_mass = mass;
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
	printf(" MY id %i", (int)m_ShapeId);
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
