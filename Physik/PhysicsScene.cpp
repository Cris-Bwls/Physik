#include "PhysicsScene.h"
#include "PhysicsObject.h"
#include <list>
#include "RigidBody.h"
#include <iostream>
#include "Plane.h"
#include "Sphere.h"
#include "Box.h"

#define DEBUG_FREQ 5

typedef CollisionInfo(*CollisionTest)(PhysicsObject*, PhysicsObject*);

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

CollisionInfo PhysicsScene::plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;
	return result;
}

CollisionInfo PhysicsScene::plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

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
			result.collNormal = collisionNormal;
			sphere2->setPosition(sphere2->getPosition() - normalize(sphere2->getVelocity()) * intersection);
			result.bCollision = true;			

			plane1->resolveCollision(sphere2, result.collNormal);

			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::plane2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

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
			result.collNormal = collisionNormal;
			result.bCollision = true;

			float test;
			if (bLBRTCollision)
				test = min(abs(fLBVerticeToPlane), abs(fRTVerticeToPlane));
			else
				test = min(abs(fLTVerticeToPlane), abs(fRBVerticeToPlane));

			box2->setPosition(box2->getPosition() - normalize(box2->getVelocity()) * test);

			plane1->resolveCollision(box2, result.collNormal);

			//TEST
			box2->SetIsFilled(!box2->GetIsFilled());

			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Plane)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Plane* plane2 = (Plane*)obj2;

		return plane2Sphere(plane2, sphere1);
	}
	return result;
}

CollisionInfo PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Sphere)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Sphere* sphere2 = (Sphere*)obj2;

		float seperation = glm::distance(sphere1->getPosition(), sphere2->getPosition());
		float fRadiusSum = sphere1->getRadius() + sphere2->getRadius();

		// IF collision
		if (seperation < fRadiusSum)
		{
			result.collNormal = normalize(sphere1->getPosition() - sphere2->getPosition());
			result.bCollision = true;

			float distance = seperation - fRadiusSum;

			float sphere1Energy = length(sphere1->getVelocity() * sphere1->getMass());
			float sphere2Energy = length(sphere2->getVelocity() * sphere2->getMass());

			float ratio = sphere1Energy / (sphere1Energy + sphere2Energy);

			auto restitute = sphere1;
			vec2 offset = normalize(restitute->getVelocity()) * distance * ratio;
			restitute->setPosition(restitute->getPosition() + offset);

			restitute = sphere2;
			offset = normalize(restitute->getVelocity()) * distance * (1 - ratio);
			restitute->setPosition(restitute->getPosition() + offset);



			float debug = length(offset);
			if (debug > 1.0f)
			{
				debug = offset.x;
			}
			else
			{
				debug = offset.y;
			}				

			sphere1->resolveCollision(sphere2, result.collNormal);

			return result;
		}
	}

	return result;
}

CollisionInfo PhysicsScene::sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Sphere && obj2->getShapeID() == ShapeID::Box)
	{
		Sphere* sphere1 = (Sphere*)obj1;
		Box* box2 = (Box*)obj2;

		vec2 boxMax = box2->getPosition() + box2->getExtents();
		vec2 boxMin = box2->getPosition() - box2->getExtents();

		vec2 spherePos = sphere1->getPosition();
		vec2 v2Clamp = clamp(spherePos, boxMin, boxMax);

		v2Clamp -= spherePos;		
		if (length(v2Clamp) <= sphere1->getRadius())
		{

			float distance = length(v2Clamp) - sphere1->getRadius();

			float sphere1Energy = length(sphere1->getVelocity() * sphere1->getMass());
			float box2Energy = length(box2->getVelocity() * box2->getMass());

			float ratio;
			if (abs(sphere1Energy) > 0)
				ratio = sphere1Energy / (sphere1Energy + box2Energy);
			else
				ratio = 0;

			Rigidbody* restitute = (Rigidbody*)sphere1;
			vec2 offset = normalize(restitute->getVelocity()) * distance * ratio;
			if (abs(offset.x) == INFINITY)

			restitute->setPosition(restitute->getPosition() + offset);
			
			restitute = (Rigidbody*)box2;
			offset = normalize(restitute->getVelocity()) * distance * (1.0f - ratio);
			restitute->setPosition(restitute->getPosition() + offset);

			result.collNormal = normalize(v2Clamp);
			result.bCollision = true;

			sphere1->resolveCollision(box2, result.collNormal);

			//TEST
			box2->SetIsFilled(!box2->GetIsFilled());

			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::box2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Plane)
	{
		Box* box1 = (Box*)obj1;
		Plane* plane2 = (Plane*)obj2;

		return plane2Box(plane2, box1);
	}
	return result;
}

CollisionInfo PhysicsScene::box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Sphere)
	{
		Box* box1 = (Box*)obj1;
		Sphere* sphere2 = (Sphere*)obj2;

		return sphere2Box(sphere2, box1);
	}
	return result;
}

CollisionInfo PhysicsScene::box2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	CollisionInfo result;
	result.bCollision = false;

	if (obj1->getShapeID() == ShapeID::Box && obj2->getShapeID() == ShapeID::Box)
	{
		Box* box1 = (Box*)obj1;
		Box* box2 = (Box*)obj2;

		vec2 box1Pos = box1->getPosition();
		vec2 box2Pos = box2->getPosition();

		vec2 box1Min = box1Pos - box1->getExtents();
		vec2 box1Max = box1Pos + box1->getExtents();
		vec2 box2Min = box2Pos - box2->getExtents();
		vec2 box2Max = box2Pos + box2->getExtents();

		static const vec2 faces[4] =
		{
			vec2(-1,  0), // 'left' face normal (-x direction)
			vec2(1,  0), // 'right' face normal (+x direction)
			vec2(0, -1), // 'bottom' face normal (-y direction)
			vec2(0,  1), // 'top' face normal (+y direction)
		};

		float distances[4] =
		{
			(box2Max.x - box1Min.x), // distance of box 'b' to face on 'left' side of 'a'.
			(box1Max.x - box2Min.x), // distance of box 'b' to face on 'right' side of 'a'.
			(box2Max.y - box1Min.y), // distance of box 'b' to face on 'bottom' side of 'a'.
			(box1Max.y - box2Min.y), // distance of box 'b' to face on 'top' side of 'a'.
		};

		vec2 collisionNormal;
		float pen;

		for (int i = 0; i < 4; i++)
		{
			// box does not intersect face. So boxes don't intersect at all.
			if (distances[i] < 0.0f)
				return result;

			// face of least intersection depth. That's our candidate.
			if ((i == 0) || (distances[i] < pen))
			{
				collisionNormal = faces[i];
				pen = distances[i];
			}
		}

		float box1Energy = length(box1->getVelocity() * box1->getMass());
		float box2Energy = length(box2->getVelocity() * box2->getMass());

		float ratio;
		if (abs(box1Energy) > 0)
			ratio = box1Energy / (box1Energy + box2Energy);
		else
			ratio = 0;

		Rigidbody* restitute = (Rigidbody*)box1;
		vec2 offset = normalize(restitute->getVelocity()) * pen * ratio;
		restitute->setPosition(restitute->getPosition() + offset);

		restitute = (Rigidbody*)box2;
		offset = normalize(restitute->getVelocity()) * pen * (1.0f - ratio);
		restitute->setPosition(restitute->getPosition() + offset);

		result.collNormal = collisionNormal;
		result.bCollision = true;
			   
		box1->resolveCollision(box2, result.collNormal);

		//TEST
		box1->SetIsFilled(!box1->GetIsFilled());
		box2->SetIsFilled(!box2->GetIsFilled());

		return result;
	}
	return result;
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
