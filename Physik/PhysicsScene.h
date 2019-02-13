#pragma once

#include <glm/ext.hpp>
#include <vector>


using std::vector;



class PhysicsObject;

struct CollisionInfo
{
	bool bCollision;
	glm::vec2 collNormal;
};

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

	static CollisionInfo plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2); 
	static CollisionInfo plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo plane2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Box(PhysicsObject* obj1, PhysicsObject* obj2);

	void debugScene();
protected:
	glm::vec2 m_gravity;
	float m_timeStep;
	vector<PhysicsObject*> m_actors;

	int debugCount = 0;
};
