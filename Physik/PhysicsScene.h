#pragma once

#include <glm/ext.hpp>
#include <vector>

using std::vector;

class PhysicsObject;

class PhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();
	void AddActor(PhysicsObject* actor);
	bool RemoveActor(PhysicsObject* actor);
	void Update(float dt);
	void UpdateGizmos();
	void setGravity(const glm::vec2 gravity) { m_gravity = gravity; }
	glm::vec2 getGravity() const { return m_gravity; }
	void setTimeStep(const float timeStep) { m_timeStep = timeStep; }
	float getTimeStep() const { return m_timeStep; }
protected:
	glm::vec2 m_gravity;
	float m_timeStep;
	vector<PhysicsObject*> m_actors;
};
