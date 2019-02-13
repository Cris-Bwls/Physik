#pragma once
#include "PhysicsObject.h"
class Rigidbody : public PhysicsObject {
public:
	Rigidbody(ShapeID shapeID, glm::vec2 position,
		glm::vec2 velocity, float rotation, float mass, float elasticity);
	~Rigidbody();

	virtual void fixedUpdate(glm::vec2 gravity, float timeStep);
	virtual void debug();

	void applyForce(glm::vec2 force);
	void applyForceToActor(Rigidbody* actor2, glm::vec2 force);

	virtual bool checkCollision(PhysicsObject* pOther) = 0;

	inline glm::vec2 getPosition() const { return m_position; }
	inline float getRotation() const { return m_rotation; }
	inline glm::vec2 getVelocity() const { return m_velocity; }
	inline float getMass() const { return m_mass; }
	inline float getElasticity() const { return m_elasticity; };
protected:
	glm::vec2 m_position;
	glm::vec2 m_velocity;
	float m_mass;
	float m_rotation; //2D so we only need a single float to represent our rotation
	float m_elasticity;
};

