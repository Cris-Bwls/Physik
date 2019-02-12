#include "PhysicsScene.h"
#include "PhysicsObject.h"
#include <list>
#include "RigidBody.h"
#include <iostream>
#include "Plane.h"
#include "Sphere.h"
#include "Box.h"

#define DEBUG_FREQ 5

typedef bool(*CollisionTest)(PhysicsObject*, PhysicsObject*);

static CollisionTest collisionFuncs[(int)ShapeID::TOTAL][(int)ShapeID::TOTAL] =
{ 
{PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere, PhysicsScene::plane2Box},
{PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::plane2Box},
{PhysicsScene::box2Plane, PhysicsScene::box2Sphere, PhysicsScene::box2Box}
};

PhysicsScene::PhysicsScene()
{
	m_timeStep = 0.01f;
	m_gravity = { 0,0 };
}


PhysicsScene::~PhysicsScene()
{
	for (auto pActor : m_actors)
	{
		delete pActor;
	}
}

void PhysicsScene::AddActor(PhysicsObject* actor)
{
	m_actors.push_back(actor);
}

bool PhysicsScene::RemoveActor(PhysicsObject* actor)
{
	for (int i = 0; i < m_actors.size(); ++i)
	{
		if (actor == m_actors[i])
		{
			m_actors.erase(m_actors.begin() + i);
			return true;
		}
	}

	return false;
}

void PhysicsScene::Update(float dt)
{
	debugScene();

	static float accumulatedTime = 0.0f;
	accumulatedTime += dt;

	static std::list<PhysicsObject*> dirty;

	while (accumulatedTime >= m_timeStep)
	{
		for each (PhysicsObject* actor in m_actors)
		{
			actor->fixedUpdate(m_gravity, m_timeStep);
		}

		accumulatedTime -= m_timeStep;


		// check for collisions (ideally you'd want to have some sort of
		// scene management in place)

		checkForCollision();

		/*for (auto pActor : m_actors) {
			for (auto pOther : m_actors) {
				if (pActor == pOther)
					continue;
				if (std::find(dirty.begin(), dirty.end(), pActor) != dirty.end() &&
					std::find(dirty.begin(), dirty.end(), pOther) != dirty.end())
					continue;
				Rigidbody* pRigid = dynamic_cast<Rigidbody*>(pActor);
				if (pRigid->checkCollision(pOther) == true) {
					pRigid->applyForceToActor(
						dynamic_cast<Rigidbody*>(pOther),
						pRigid->getVelocity() * pRigid->getMass());
					dirty.push_back(pRigid);
					dirty.push_back(pOther);
				}
			}
		}
		dirty.clear();*/
	}
}

void PhysicsScene::UpdateGizmos()
{
	for each (PhysicsObject* actor in m_actors)
	{
		actor->makeGizmo();
	}
}

void PhysicsScene::checkForCollision()
{
	int actorCount = m_actors.size();

	for (int outer = 0; outer < actorCount - 1; ++outer)
	{
		for (int inner = outer + 1; inner < actorCount; inner++)
		{
			PhysicsObject* object1 = m_actors[outer];
			PhysicsObject* object2 = m_actors[inner];
			int shapeID1 = (int)object1->getShapeID();
			int shapeID2 = (int)object2->getShapeID();

			auto collisionFuncPtr = collisionFuncs[shapeID1][shapeID2];
			if (collisionFuncPtr)
			{
				collisionFuncPtr(object1, object2);
			}
		}
	}
}

bool PhysicsScene::plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Plane && obj2->getShapeID() == ShapeID::Plane)
	{
		Plane* plane1 = (Plane*)obj1;
		Plane* plane2 = (Plane*)obj2;

		float fDot = dot(plane1->getNormal(), plane2->getNormal());
		float fOriginSeperation = plane1->getDistance() - plane2->getDistance();

		if (abs(fDot) != 1.0f || fOriginSeperation == 0)
		{
			return true;
		}
	}
	return false;
}

bool PhysicsScene::plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Plane && obj2->getShapeID() == ShapeID::Sphere)
	{
		Plane* plane1 = (Plane*)obj1;
		Sphere* sphere2 = (Sphere*)obj2;

		vec2 collisionNormal = plane1->getNormal();
		float sphereToPlane = dot(sphere2->getPosition(), plane1->getNormal()) - plane1->getDistance();

		if (sphereToPlane < 0)
		{
			collisionNormal *= -1;
			sphereToPlane *= -1;
		}

		float intersection = sphere2->getRadius() - sphereToPlane;
		if (intersection > 0)
		{
			auto vel2 = sphere2->getVelocity();
			sphere2->applyForce(-vel2 * sphere2->getMass());

			return true;
		}
	}
	return false;
}

bool PhysicsScene::plane2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Plane && obj2->getShapeID() == ShapeID::Box)
	{
		Plane* plane1 = (Plane*)obj1;
		Box* box2 = (Box*)obj2;

		vec2 collisionNormal = plane1->getNormal();

		vec2 verticeLB = box2->getPosition() - box2->getExtents();
		vec2 verticeRT = box2->getPosition() + box2->getExtents();
		
		vec2 verticeLT = { verticeLB.x, verticeRT.y };
		vec2 verticeRB = { verticeRT.x, verticeLB.y };

		float fLBVerticeToPlane = dot(verticeLB, plane1->getNormal()) - plane1->getDistance();
		float fRTVerticeToPlane = dot(verticeRT, plane1->getNormal()) - plane1->getDistance();

		float fLTVerticeToPlane = dot(verticeLT, plane1->getNormal()) - plane1->getDistance();
		float fRBVerticeToPlane = dot(verticeRB, plane1->getNormal()) - plane1->getDistance();

		bool bLBRTCollision = fLBVerticeToPlane < 0 && fRTVerticeToPlane > 0;
		bool bLTRBCollision = fLTVerticeToPlane > 0 && fRBVerticeToPlane < 0;

		if (bLBRTCollision || bLTRBCollision)
		{
			//COLLISION
			auto vel2 = box2->getVelocity();
			box2->applyForce(-vel2 * box2->getMass());

			return true;
		}
	}
	return false;
}

bool PhysicsScene::sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Plane)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Plane* plane2 = (Plane*)obj2;

		return plane2Sphere(plane2, sphere1);
	}
	return false;
}

bool PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Sphere)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Sphere* sphere2 = (Sphere*)obj2;

		float seperation = glm::distance(sphere1->getPosition(), sphere2->getPosition());
		float fRadiusSum = sphere1->getRadius() + sphere2->getRadius();

		// IF collision
		if (seperation < fRadiusSum)
		{
			auto vel1 = sphere1->getVelocity();
			auto vel2 = sphere2->getVelocity();

			sphere1->applyForce(-vel1 * sphere1->getMass());
			sphere2->applyForce(-vel2 * sphere2->getMass());

			return true;
		}
	}

	return false;
}

bool PhysicsScene::sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Box)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Box* box2 = (Box*)obj2;

		vec2 boxMax = box2->getPosition() + box2->getExtents();
		vec2 boxMin = box2->getPosition() - box2->getExtents();

		vec2 spherePos = sphere1->getPosition();
		vec2 clamp = spherePos;

		bool xIn = false;
		bool yIn = false;

		if (spherePos.x < boxMin.x)
		{
			clamp.x = boxMin.x;
		}
		else if (spherePos.x > boxMax.x)
		{
			clamp.x = boxMax.x;
		}
		else
		{
			xIn = true;
		}

		if (spherePos.y < boxMin.y)
		{
			clamp.y = boxMin.y;
		}
		else if (spherePos.y > boxMax.y)
		{
			clamp.y = boxMax.y;
		}
		else
		{
			yIn = true;
		}

		if (xIn && yIn)
		{
			auto vel1 = sphere1->getVelocity();
			auto vel2 = box2->getVelocity();

			sphere1->applyForce(-vel1 * sphere1->getMass());
			box2->applyForce(-vel2 * box2->getMass());
			return true;
		}

		clamp -= spherePos;		
		if (length(clamp) <= sphere1->getRadius())
		{
			auto vel1 = sphere1->getVelocity();
			auto vel2 = box2->getVelocity();

			sphere1->applyForce(-vel1 * sphere1->getMass());
			box2->applyForce(-vel2 * box2->getMass());
			return true;
		}
	}
	return false;
}

bool PhysicsScene::box2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Plane)
	{
		Box* box1 = (Box*)obj1;
		Plane* plane2 = (Plane*)obj2;

		return plane2Box(plane2, box1);
	}
	return false;
}

bool PhysicsScene::box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Sphere)
	{
		Box* box1 = (Box*)obj1;
		Sphere* sphere2 = (Sphere*)obj2;

		return sphere2Box(sphere2, box1);
	}
	return false;
}

bool PhysicsScene::box2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Box)
	{
		Box* box1 = (Box*)obj1;
		Box* box2 = (Box*)obj2;

		vec2 max1 = box1->getPosition() + box1->getExtents();
		vec2 min1 = box1->getPosition() - box1->getExtents();

		vec2 max2 = box2->getPosition() + box2->getExtents();
		vec2 min2 = box2->getPosition() - box2->getExtents();

		bool checkX = max1.x < min2.x || max2.x < min1.x;
		bool checkY = max1.y < min2.y || max2.y < min1.y;

		if (!(checkX || checkY))
		{
			auto vel1 = box1->getVelocity();
			auto vel2 = box2->getVelocity();

			box1->applyForce(-vel1 * box1->getMass());
			box2->applyForce(-vel2 * box2->getMass());

			return true;
		}
	}
	return false;
}

void PhysicsScene::debugScene()
{
	if (debugCount == 0)
	{
		system("CLS");
		int count = 0;
		for (auto pActor : m_actors) {
			std::cout << count << " : ";
			pActor->debug();
			count++;
		}
	}

	debugCount++;
	if (debugCount > DEBUG_FREQ)
		debugCount = 0;
}
