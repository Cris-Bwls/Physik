#pragma once
#include "PhysicsObject.h"
#include <glm/ext.hpp>
#include "RigidBody.h"

using namespace glm;

class Plane : public PhysicsObject
{
public:
	Plane(glm::vec2 normal, float distance, float fFricCoStatic, float fFricCoDynamic);
	~Plane();

	virtual void fixedUpdate(vec2 const& gravity, float timeStep) {};
	virtual void makeGizmo();
	virtual	void resetPosition();
	virtual void debug() {};

	inline vec2 getNormal() { return m_normal; };
	inline float getDistance() { return m_distanceToOrigin; };

	void Plane::resolveCollision(RigidBody* actor2, vec2 const& normal);
	
protected:
	vec2 m_normal;
	float m_distanceToOrigin;
};

