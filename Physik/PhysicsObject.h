#pragma once
#include <glm/ext.hpp>

enum class ShapeID : int
{
	Plane = 0,
	Sphere,
	Box,

	TOTAL
};

class PhysicsObject
{
public:
	virtual void fixedUpdate(glm::vec2 const& gravity, float timeStep) = 0;
	virtual void debug() = 0;
	virtual void makeGizmo() = 0;
	virtual void resetPosition() {};

	inline ShapeID getShapeID() { return m_ShapeId; };

protected:
	PhysicsObject(ShapeID shapeID) : m_ShapeId(shapeID) {};
	ShapeID m_ShapeId;
};

