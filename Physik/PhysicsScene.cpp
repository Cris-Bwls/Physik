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
{PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere, PhysicsScene::plane2Box, PhysicsScene::plane2Poly},
{PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::sphere2Box, PhysicsScene::sphere2Poly},
{PhysicsScene::box2Plane, PhysicsScene::box2Sphere, PhysicsScene::box2Box, PhysicsScene::box2Poly}
};

PhysicsScene::PhysicsScene()
{
	m_timeStep = 0.01f;
	m_gravity = { 0,0 };

	m_recordedOffset = 0;
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
	//debugScene();

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
				auto info = collisionFuncPtr(object1, object2);
				if (info.bCollision)
				{
					if (shapeID1 == (int)ShapeID::Plane)
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object2);
						((Plane*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);
					}
					else if (shapeID2 == (int)ShapeID::Plane)
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1);
						((Plane*)object2)->resolveCollision((RigidBody*)object2, info.collNormal);
					}
					else
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1, (RigidBody*)object2);
						((RigidBody*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);
					}
				}
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
			result.bCollision = true;
			result.fPenetration = intersection;

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
			float pen;
			if (bLBRTCollision)
				pen = min(abs(fLBVerticeToPlane), abs(fRTVerticeToPlane));
			else
				pen = min(abs(fLTVerticeToPlane), abs(fRBVerticeToPlane));

			result.collNormal = collisionNormal;
			result.bCollision = true;
			result.fPenetration = pen;

			//TEST
			box2->SetIsFilled(!box2->GetIsFilled());

			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::plane2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return CollisionInfo();
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
			float pen = seperation - fRadiusSum;

			result.fPenetration = pen;
			result.collNormal = normalize(sphere1->getPosition() - sphere2->getPosition());
			result.bCollision = true;
			result.fPenetration = pen;

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
			float pen = length(v2Clamp) - sphere1->getRadius();
			
			if (v2Clamp == vec2(0, 0))
				result.collNormal = normalize(spherePos - box2->getPosition());
			else
				result.collNormal = normalize(v2Clamp);

			result.bCollision = true;
			result.fPenetration = pen;

			if (pen != pen)
				printf("FUCK");

			//TEST
			box2->SetIsFilled(!box2->GetIsFilled());

			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::sphere2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return CollisionInfo();
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
			if (distances[i] < 0.0f || distances[i] != distances[i])
				return result;

			// face of least intersection depth. That's our candidate.
			if ((i == 0) || (distances[i] < pen))
			{
				collisionNormal = faces[i];
				pen = distances[i];
			}
		}

		result.collNormal = collisionNormal;
		result.bCollision = true;
		result.fPenetration = pen;

		//TEST
		box1->SetIsFilled(!box1->GetIsFilled());
		box2->SetIsFilled(!box2->GetIsFilled());

		return result;
	}
	return result;
}

CollisionInfo PhysicsScene::box2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return CollisionInfo();
}

void PhysicsScene::Restitution(float overlap, glm::vec2 const& collNormal, RigidBody * rb1, RigidBody * rb2)
{
	overlap = abs(overlap);
	if (overlap < 0.01f || overlap != overlap)
		return;

	bool debug = false;
	bool rb1GoodVel = true;
	bool rb2GoodVel = true;

	float ratio = 1;

	// check velocity is real
	vec2 rb1Vel = rb1->getVelocity();
	float rb1Speed = length(rb1Vel);
	if (rb1Speed == 0 || rb1Speed != rb1Speed)
	{
		rb1GoodVel = false;
		rb1Vel = collNormal;
		rb1Speed = 0;
	}
	
	// IF rb2 exists
	if (rb2)
	{
		// check velocity is real
		vec2 rb2Vel = rb2->getVelocity();
		float rb2Speed = length(rb2Vel);
		if (rb2Speed == 0 || rb2Speed != rb2Speed)
		{
			rb2GoodVel = false;
			rb2Vel = -collNormal;
			rb2Speed = 0;
		}

		if (rb1GoodVel + rb2GoodVel == false)
		{
			rb1Speed = 1, rb2Speed = 1;
		}

		// Get the real ratio
		float rb1Mom = rb1Speed * rb1->getMass();
		float rb2Mom = rb2Speed * rb2->getMass();

		float ratio = rb1Mom / (rb1Mom + rb2Mom);
	
		// Restitute rb2
		vec2 offset = normalize(rb2Vel) * overlap * (1 - ratio);
		rb2->setPosition(rb2->getPosition() - offset);

		float offsetMag = length(offset);
		if (offsetMag > m_recordedOffset)
			m_recordedOffset = offsetMag;
		if (offsetMag > 0.5f)
			debug = true;
	}

	// Restitute rb1
	vec2 offset = normalize(rb1Vel) * overlap * ratio;
	rb1->setPosition(rb1->getPosition() - offset);

	float offsetMag = length(offset);
	if (offsetMag > m_recordedOffset)
		m_recordedOffset = offsetMag;
	if (offsetMag > 0.5f)
			debug = true;
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
			printf("\n");
			count++;
		}

		std::cout << count << "Recorded Offset = " << m_recordedOffset;
	}

	debugCount++;
	if (debugCount > DEBUG_FREQ)
		debugCount = 0;
}
