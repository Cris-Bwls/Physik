#pragma once
#include <glm/ext.hpp>

enum class ShapeID : int
{
	Plane,
	Sphere,
	Box
};

class PhysicsObject
{
public:
	virtual void fixedUpdate(glm::vec2 gravity, float timeStep) = 0;
	virtual void debug() = 0;
	virtual void makeGizmo() = 0;
	virtual void resetPosition() {};

protected:
	PhysicsObject(ShapeID shapeID) : m_ShapeId(shapeID) {};
	ShapeID m_ShapeId;
};

