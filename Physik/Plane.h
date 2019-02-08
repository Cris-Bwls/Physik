#pragma once
#include "PhysicsObject.h"
#include <glm/ext.hpp>

using namespace glm;

class Plane : public PhysicsObject
{
public:
	Plane();
	Plane(glm::vec2 normal, float distance);
	~Plane();

	virtual void fixedUpdate(vec2 gravity, float timeStep);
	virtual void makeGizmo();
	virtual	void resetPosition();
	virtual void debug() {};

	inline vec2 getNormal() { return m_normal; };
	inline float getDistance() { return m_distanceToOrigin; };

protected:
	vec2 m_normal;
	float m_distanceToOrigin;
};

