#pragma once
#include "RigidBody.h"

class Box :
	public Rigidbody
{
public:
	Box();
	Box(glm::vec2 extents, glm::vec2 position, glm::vec2 velocity, float mass, float elasticity, glm::vec4 colour, bool bIsFilled);
	~Box();

	virtual void makeGizmo();

	inline bool checkCollision(PhysicsObject* pOther) { return false; }

	inline glm::vec2 getExtents() { return m_Extents; };
	inline glm::vec4 getColour() { return m_Colour; };

	inline bool GetIsFilled() { return m_bIsFilled; };
	inline void SetIsFilled(bool bIsFilled) { m_bIsFilled = bIsFilled; };

protected:
	glm::vec2 m_Extents;
	glm::vec4 m_Colour;
	bool m_bIsFilled;
};

