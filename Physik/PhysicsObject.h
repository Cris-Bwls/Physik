#pragma once
#include <glm/ext.hpp>

enum class ShapeID : int
{
	Plane = 0,
	Sphere,
	Box,
	Poly,
	Stitched,

	TOTAL
};

class PhysicsObject
{
public:
	inline virtual ~PhysicsObject() {};

	virtual void fixedUpdate(glm::vec2 const& gravity, float timeStep) = 0;
	virtual void debug() = 0;
	virtual void makeGizmo() = 0;
	virtual void resetPosition() {};

	inline ShapeID getShapeID() { return m_ShapeId; };
	inline float GetStaticFricCo() const { return m_fFricCoStatic; };
	inline float GetKineticFricCo() const { return m_fFricCoKinetic; };

protected:
	inline PhysicsObject(ShapeID shapeID, float fFricCoStatic, float fFricCoDynamic) 
		: m_ShapeId(shapeID), m_fFricCoStatic(fFricCoStatic), m_fFricCoKinetic(fFricCoDynamic) {};
	ShapeID m_ShapeId;

	float m_fFricCoStatic;
	float m_fFricCoKinetic;
};

