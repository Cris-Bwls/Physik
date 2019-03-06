#pragma once

#include <glm/ext.hpp>
#include <vector>


using std::vector;

class PhysicsObject;
class RigidBody;
class Plane;

struct CollisionInfo
{
	bool bCollision = false;
	glm::vec2 collNormal;
	float fPenetration;
	glm::vec2 collPoint;
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
	static CollisionInfo plane2Poly(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo plane2Stitched(PhysicsObject* obj1, PhysicsObject* obj2);

	static CollisionInfo sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Poly(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo sphere2Stitched(PhysicsObject* obj1, PhysicsObject* obj2);

	static CollisionInfo box2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Poly(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo box2Stitched(PhysicsObject* obj1, PhysicsObject* obj2);

	static CollisionInfo poly2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo poly2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo poly2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo poly2Poly(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo poly2Stitched(PhysicsObject* obj1, PhysicsObject* obj2);

	static CollisionInfo stitched2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo stitched2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo stitched2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo stitched2Poly(PhysicsObject* obj1, PhysicsObject* obj2);
	static CollisionInfo stitched2Stitched(PhysicsObject* obj1, PhysicsObject* obj2);

	void Restitution(float overlap, glm::vec2 const& collNormal, RigidBody* rb1, RigidBody* rb2 = nullptr);
	void CalcTorque(glm::vec2 const& collPoint, RigidBody* rb1, RigidBody* rb2 = nullptr);

	void debugScene();
protected:
	static bool ProjectionOverlap(float const& min1, float const& max1, float const& min2, float const& max2, float & overlap);
	static float DistPointPlane(glm::vec2 const& point, Plane* plane);
	static glm::vec2 ClosestPtPointPlane(glm::vec2 const& point, Plane* plane);
	static void ClosestPtPointSegment(glm::vec2 const& point, glm::vec2 const& start, glm::vec2 end, float &t, glm::vec2 &closest);
	static float ClosestPtSegmentSegment(glm::vec2 const& start1, glm::vec2 const& start2, glm::vec2 const& end1, glm::vec2 const& end2, 
											float &s, float &t, glm::vec2 &closest1, glm::vec2 &closest2);
	static float Clamp(float n, float min, float max);

	glm::vec2 m_gravity;
	float m_timeStep;
	vector<PhysicsObject*> m_actors;

	float time = 0;
	int debugCount = 0;	
};
