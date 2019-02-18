#pragma once
#include "PhysicsObject.h"
class RigidBody : public PhysicsObject {
public:
	RigidBody(ShapeID shapeID, glm::vec2 position,
		glm::vec2 velocity, float rotation, float mass, float elasticity);
	~RigidBody();

	virtual void fixedUpdate(glm::vec2 const& gravity, float timeStep);
	virtual void debug();

	void applyForce(glm::vec2 const& force);
	void applyForceToActor(RigidBody* actor2, glm::vec2 const& force);

	virtual bool checkCollision(PhysicsObject* pOther) = 0;

	inline void setPosition(glm::vec2 const& pos) { m_position = pos; }
	inline glm::vec2 getPosition() const { return m_position; }
	inline glm::vec2 getPrevPosition() const { return m_prevPosition; }
	inline float getRotation() const { return m_rotation; }
	inline glm::vec2 getVelocity() const { return m_velocity; }
	inline float getMass() const { return m_mass; }
	inline float getElasticity() const { return m_elasticity; };

	
	void resolveCollision(RigidBody* actor2, glm::vec2 const& normal);
protected:
	void DebugVelocity(glm::vec2 const& startPoint);

	glm::vec2 m_position;
	glm::vec2 m_prevPosition;
	glm::vec2 m_velocity;
	float m_mass;
	float m_rotation; //2D so we only need a single float to represent our rotation
	float m_elasticity;
};

