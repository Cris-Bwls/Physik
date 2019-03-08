#pragma once
#include "PhysicsObject.h"
class RigidBody : public PhysicsObject {
public:
	RigidBody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float fAngVelocity, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag);
	virtual ~RigidBody();

	virtual void fixedUpdate(glm::vec2 const& gravity, float timeStep);
	virtual void debug();

	void applyForce(glm::vec2 const& force);
	void applyForceToActor(RigidBody* actor2, glm::vec2 const& force);

	//virtual bool checkCollision(PhysicsObject* pOther) = 0;

	inline void AddResolutionForce(glm::vec2 const& force) { m_ResolutionForceSum += force; };
	inline void AddResolutionForceToActor(RigidBody* actor2, glm::vec2 const& force) { AddResolutionForce(force); actor2->AddResolutionForce(-force); }
	inline void ApplyResolutionForce() { applyForce(m_ResolutionForceSum); m_ResolutionForceSum = { 0,0 }; };

	inline void setPosition(glm::vec2 const& pos) { m_position = pos; }
	inline glm::vec2 getPosition() const { return m_position; }
	inline void setRotation(float const& rot) { m_rotation = rot; };
	inline float getRotation() const { return m_rotation; }
	inline void setVelocity(glm::vec2 const& velocity) { m_velocity = velocity; };
	inline glm::vec2 getVelocity() const { return m_velocity; }
	inline float getMass() const { return m_mass; }
	inline float getElasticity() const { return m_elasticity; };
	inline void setAngularVelocity(float const& angVel) { m_angularVelocity = angVel; };
	inline float getAngularVelocity() const { return m_angularVelocity; };
	inline void setAngularDrag(float const& angDrag) { m_angularDrag = angDrag; };
	inline float getAngularDrag() const { return m_angularDrag; };
	inline void setDrag(float const& drag) { m_drag = drag; };
	inline float getDrag() const { return m_drag; };

	inline bool GetIsFilled() const { return m_bIsFilled; };
	inline void SetIsFilled(bool const& bIsFilled) { m_bIsFilled = bIsFilled; };
	inline void InvertIsFilled() { m_bIsFilled = !m_bIsFilled; };
	
	void resolveCollision(RigidBody* actor2, glm::vec2 const& normal);
protected:
	void ApplyDrags(float const& timeStep);
	void DebugVelocity(glm::vec2 const& startPoint);

	glm::vec2 m_position;
	glm::vec2 m_ResolutionForceSum;
	glm::vec2 m_velocity;
	float m_mass;
	float m_rotation;
	float m_elasticity;

	float m_angularVelocity = 0;

	float m_drag;
	float m_angularDrag;

	bool m_bIsFilled;
};

