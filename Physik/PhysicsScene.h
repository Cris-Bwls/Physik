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
	float getTimeStep() const { return m_timeStep; };

	void checkForCollision(); 

	static bool plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2); 
	static bool plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool plane2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool box2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool box2Box(PhysicsObject* obj1, PhysicsObject* obj2);

	void debugScene();
protected:
	glm::vec2 m_gravity;
	float m_timeStep;
	vector<PhysicsObject*> m_actors;

	int debugCount = 0;
};
