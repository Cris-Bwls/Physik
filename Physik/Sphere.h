#pragma once
#include "RigidBody.h"

class Sphere : public RigidBody
{
public:
	Sphere(glm::vec2 position, glm::vec2 velocity, float fAngRot, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag, float radius, glm::vec4 colour);
	~Sphere();
	virtual void makeGizmo();
	virtual bool checkCollision(PhysicsObject* pOther);
	inline float getRadius() { return m_radius; }
	inline glm::vec4 getColour() { return m_colour; }

	inline void HideDirLine() { m_bDirLine = false; };
protected:
	float m_radius;
	glm::vec4 m_colour;

	bool m_bDirLine = true;
};

