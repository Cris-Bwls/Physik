#include "PhysicsScene.h"
#include "PhysicsObject.h"
#include <list>
#include "RigidBody.h"
#include <iostream>
#include "Plane.h"
#include "Sphere.h"
#include "Box.h"
#include "Poly.h"
#include "Stitched.h"

#define DEBUG_FREQ 5

typedef CollisionInfo(*CollisionTest)(PhysicsObject*, PhysicsObject*);

static CollisionTest collisionFuncs[(int)ShapeID::TOTAL][(int)ShapeID::TOTAL] =
{ 
{PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere, PhysicsScene::plane2Box, PhysicsScene::plane2Poly, PhysicsScene::plane2Stitched},
{PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::sphere2Box, PhysicsScene::sphere2Poly, PhysicsScene::sphere2Stitched},
{PhysicsScene::box2Plane, PhysicsScene::box2Sphere, PhysicsScene::box2Box, PhysicsScene::box2Poly, PhysicsScene::box2Stitched},
{PhysicsScene::poly2Plane, PhysicsScene::poly2Sphere, PhysicsScene::poly2Box, PhysicsScene::poly2Poly, PhysicsScene::poly2Stitched},
{PhysicsScene::stitched2Plane, PhysicsScene::stitched2Sphere, PhysicsScene::stitched2Box, PhysicsScene::stitched2Poly, PhysicsScene::stitched2Stitched}
};

PhysicsScene::PhysicsScene()
{
	m_timeStep = 0.01f;
	m_gravity = { 0,0 };
}


PhysicsScene::~PhysicsScene()
{
	for (int i = 0; i < m_actors.size(); ++i)
	{
		delete m_actors[i];
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

	while (accumulatedTime >= m_timeStep)
	{
		time += m_timeStep;
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
				CollisionInfo info = collisionFuncPtr(object1, object2);
				if (info.bCollision)
				{
					if (shapeID1 == (int)ShapeID::Plane)
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object2);
						((Plane*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);

						// DEBUG
						((RigidBody*)object2)->InvertIsFilled();
					}
					else if (shapeID2 == (int)ShapeID::Plane)
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1);
						((Plane*)object2)->resolveCollision((RigidBody*)object2, info.collNormal);

						// DEBUG
						((RigidBody*)object1)->InvertIsFilled();
					}
					else
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1, (RigidBody*)object2);
						((RigidBody*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);

						// DEBUG
						((RigidBody*)object1)->InvertIsFilled();
						((RigidBody*)object2)->InvertIsFilled();
					}
					bool debug;
					if (info.collNormal.x != info.collNormal.x)
						debug = true;
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

		vec2 collNorm = plane1->getNormal();

		vec2 boxPos = box2->getPosition();
		vec2 boxExtent = box2->getExtents();

		float r = (boxExtent[0] * abs(collNorm[0])) + (boxExtent[1] * abs(collNorm[1]));
		float s = dot(collNorm, boxPos) - plane1->getDistance();

		float pen = r - abs(s);

		result.bCollision = pen >= 0;
		if (result.bCollision)
		{
			result.fPenetration = pen;
			result.collNormal = -collNorm;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::plane2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Plane* plane1 = (Plane*)obj1;
	Poly* poly2 = (Poly*)obj2;
	CollisionInfo broad = plane2Sphere(plane1, poly2->GetBroadColl());
	if (!broad.bCollision)
		return broad;

	// Perform SAT check
	CollisionInfo sat;
	sat.collNormal = plane1->getNormal();

	float polyMax, polyMin;
	poly2->Project(sat.collNormal, polyMin, polyMax);

	float planeDistance = plane1->getDistance();
	sat.bCollision = ProjectionOverlap(planeDistance, planeDistance, polyMin, polyMax, sat.fPenetration);

	sat.fPenetration -= (polyMax - polyMin);

	return sat;
}

CollisionInfo PhysicsScene::plane2Stitched(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Plane* plane1 = (Plane*)obj1;
	Stitched* stitched1 = (Stitched*)obj2;

	vector<CollisionInfo> allCollInfo;
	CollisionInfo result;
	result.collNormal = { 0,0 };
	int count = stitched1->GetPolyCount();

	for (int i = 0; i < count; ++i)
	{
		auto poly = stitched1->GetPoly(i);
		allCollInfo.push_back(plane2Poly(plane1, poly));
		result.bCollision += allCollInfo[i].bCollision;
	}

	if (result.bCollision)
	{
		CollisionInfo backup;
		
		int collCount = 0;
		for (int i = 0; i < count; ++i)
		{
			if (allCollInfo[i].bCollision)
			{
				++collCount;
				result.collNormal += (allCollInfo[i].collNormal * allCollInfo[i].fPenetration);
				
				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
			result = backup;
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
			float pen = seperation - fRadiusSum;

			result.fPenetration = pen;
			result.collNormal = normalize(sphere1->getPosition() - sphere2->getPosition());
			result.bCollision = true;

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
			
			return result;
		}
	}
	return result;
}

CollisionInfo PhysicsScene::sphere2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Sphere* sphere1 = (Sphere*)obj1;
	Poly* poly2 = (Poly*)obj2;
	CollisionInfo broad = sphere2Sphere(sphere1, poly2->GetBroadColl());
	if (!broad.bCollision)
		return broad;

	// Perform SAT check
	CollisionInfo sat;
	sat.fPenetration = FLT_MAX;

	float sphereDot, sphereMax, sphereMin;
	float polyMax, polyMin;
	float overlap;

	for (int i = 0; i < poly2->GetSNormCount(); ++i)
	{
		if (poly2->GetSNormParallel(i))
			continue;

		vec2 norm = poly2->GetRotatedSNorm(i);

		sphereDot = dot(norm, sphere1->getPosition());
		sphereMax = sphereDot + sphere1->getRadius();
		sphereMin = sphereDot - sphere1->getRadius();

		poly2->Project(norm, polyMin, polyMax);

		sat.bCollision = ProjectionOverlap(sphereMin, sphereMax, polyMin, polyMax, overlap);

		//No collision EARLY EXIT
		if (!(sat.bCollision))
			return sat;

		overlap -= (polyMax - polyMin);
		overlap -= (sphereMax - sphereMin);

		overlap = abs(overlap);

		if (sat.fPenetration > overlap)
		{
			sat.fPenetration = overlap;
			sat.collNormal = norm;
		}
	}

	return sat;
}

CollisionInfo PhysicsScene::sphere2Stitched(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Sphere* sphere1 = (Sphere*)obj1;
	Stitched* stitched1 = (Stitched*)obj2;

	vector<CollisionInfo> allCollInfo;
	CollisionInfo result;
	result.collNormal = { 0,0 };
	int count = stitched1->GetPolyCount();

	for (int i = 0; i < count; ++i)
	{
		auto poly = stitched1->GetPoly(i);
		allCollInfo.push_back(sphere2Poly(sphere1, poly));
		result.bCollision += allCollInfo[i].bCollision;
	}

	if (result.bCollision)
	{
		CollisionInfo backup;

		int collCount = 0;
		for (int i = 0; i < count; ++i)
		{
			if (allCollInfo[i].bCollision)
			{
				++collCount;
				result.collNormal += (allCollInfo[i].collNormal * allCollInfo[i].fPenetration);

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
			result = backup;

		result = backup;
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
		
		return result;
	}
	return result;
}

CollisionInfo PhysicsScene::box2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Box* box1 = (Box*)obj1;
	Poly* poly2 = (Poly*)obj2;
	CollisionInfo broad = box2Sphere(box1, poly2->GetBroadColl());
	if (!broad.bCollision)
		return broad;

	// Perform SAT check
	CollisionInfo sat;
	sat.fPenetration = FLT_MAX;

	float boxMax, boxMin;
	float polyMax, polyMin;
	float overlap;

	vec2 boxPos = box1->getPosition();
	vec2 boxExtent = box1->getExtents();

	vec2 boxVerts[4] =
	{
		boxPos + boxExtent,
		boxPos + vec2(boxExtent.x, -boxExtent.y),
		boxPos + -boxExtent,
		boxPos + vec2(-boxExtent.x, boxExtent.y)
	};

	static const vec2 boxNorms[2] = { vec2(0,1), vec2(1,0) };
	for (int i = 1; i < 2; ++i)
	{
		vec2 norm = boxNorms[i];

		boxMin = dot(norm, boxVerts[2]);
		boxMax = boxMin;
		for (int i = 0; i < 4; ++i)
		{
			float temp = dot(norm, boxVerts[i]);
			if (temp < boxMin)
			{
				boxMin = temp;
			}
			else if (temp > boxMax)
			{
				boxMax = temp;
			}
		}

		poly2->Project(norm, polyMin, polyMax);

		sat.bCollision = ProjectionOverlap(boxMin, boxMax, polyMin, polyMax, overlap);

		//No collision EARLY EXIT
		if (!(sat.bCollision))
			return sat;

		overlap -= (polyMax - polyMin);
		overlap -= (boxMax - boxMin);

		overlap = abs(overlap);

		if (sat.fPenetration > overlap)
		{
			sat.fPenetration = overlap;
			sat.collNormal = norm;
		}
	}

	for (int i = 0; i < poly2->GetSNormCount(); ++i)
	{
		if (poly2->GetSNormParallel(i))
			continue;

		vec2 norm = poly2->GetRotatedSNorm(i);

		boxMin = dot(norm, boxVerts[2]);
		boxMax = boxMin;
		for (int i = 0; i < 4; ++i)
		{
			float temp = dot(norm, boxVerts[i]);
			if (temp < boxMin)
			{
				boxMin = temp;
			}
			else if (temp > boxMax)
			{
				boxMax = temp;
			}
		}

		poly2->Project(norm, polyMin, polyMax);

		sat.bCollision = ProjectionOverlap(boxMin, boxMax, polyMin, polyMax, overlap);

		//No collision EARLY EXIT
		if (!(sat.bCollision))
			return sat;

		overlap -= (polyMax - polyMin);
		overlap -= (boxMax - boxMin);

		overlap = abs(overlap);

		if (sat.fPenetration > overlap)
		{
			sat.fPenetration = overlap;
			sat.collNormal = norm;
		}
	}	

	return sat;
}

CollisionInfo PhysicsScene::box2Stitched(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Box* box1 = (Box*)obj1;
	Stitched* stitched1 = (Stitched*)obj2;

	vector<CollisionInfo> allCollInfo;
	CollisionInfo result;
	result.collNormal = { 0,0 };
	int count = stitched1->GetPolyCount();

	for (int i = 0; i < count; ++i)
	{
		auto poly = stitched1->GetPoly(i);
		allCollInfo.push_back(box2Poly(box1, poly));
		result.bCollision += allCollInfo[i].bCollision;
	}

	if (result.bCollision)
	{
		CollisionInfo backup;

		int collCount = 0;
		for (int i = 0; i < count; ++i)
		{
			if (allCollInfo[i].bCollision)
			{
				++collCount;
				result.collNormal += (allCollInfo[i].collNormal * allCollInfo[i].fPenetration);

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
			result = backup;

		result = backup;
	}
	return result;
}

CollisionInfo PhysicsScene::poly2Plane(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return plane2Poly(obj2, obj1);
}

CollisionInfo PhysicsScene::poly2Sphere(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return sphere2Poly(obj2, obj1);
}

CollisionInfo PhysicsScene::poly2Box(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return box2Poly(obj2, obj1);
}

CollisionInfo PhysicsScene::poly2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Poly* poly1 = (Poly*)obj1;
	Poly* poly2 = (Poly*)obj2;
	CollisionInfo broad = sphere2Sphere(poly1->GetBroadColl(), poly2->GetBroadColl());
	if (!broad.bCollision)
		return broad;

	// Perform SAT check
	CollisionInfo sat;
	sat.fPenetration = FLT_MAX;
	
	float poly1Max, poly1Min;
	float poly2Max, poly2Min;
	float overlap;
	
	for (int i = 0; i < poly1->GetSNormCount(); ++i)
	{
		if (poly1->GetSNormParallel(i))
			continue;

		vec2 norm = poly1->GetRotatedSNorm(i);		
	
		poly1->Project(norm, poly1Min, poly1Max);
		poly2->Project(norm, poly2Min, poly2Max);
	
		sat.bCollision = ProjectionOverlap(poly1Min, poly1Max, poly2Min, poly2Max, overlap);
	
		//No collision EARLY EXIT
		if (!(sat.bCollision))
			return sat;
	
		overlap -= (poly1Max - poly1Min);
		overlap -= (poly2Max - poly2Min);
	
		overlap = abs(overlap);
	
		if (sat.fPenetration > overlap)
		{
			sat.fPenetration = overlap;
			sat.collNormal = norm;
		}
	}
	
	for (int i = 0; i < poly2->GetSNormCount(); ++i)
	{
		if (poly2->GetSNormParallel(i))
			continue;

		vec2 norm = poly2->GetRotatedSNorm(i);
	
		poly1->Project(norm, poly1Min, poly1Max);
		poly2->Project(norm, poly2Min, poly2Max);
	
		sat.bCollision = ProjectionOverlap(poly1Min, poly1Max, poly2Min, poly2Max, overlap);
	
		//No collision EARLY EXIT
		if (!(sat.bCollision))
			return sat;
	
		overlap -= (poly1Max - poly1Min);
		overlap -= (poly2Max - poly2Min);
	
		overlap = abs(overlap);
	
		if (sat.fPenetration > overlap)
		{
			sat.fPenetration = overlap;
			sat.collNormal = norm;
		}
	}

	return sat;
}

CollisionInfo PhysicsScene::poly2Stitched(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Poly* poly1 = (Poly*)obj1;
	Stitched* stitched1 = (Stitched*)obj2;

	vector<CollisionInfo> allCollInfo;
	CollisionInfo result;
	result.collNormal = { 0,0 };
	int count = stitched1->GetPolyCount();

	for (int i = 0; i < count; ++i)
	{
		auto poly = stitched1->GetPoly(i);
		allCollInfo.push_back(poly2Poly(poly1, poly));
		result.bCollision += allCollInfo[i].bCollision;
	}

	if (result.bCollision)
	{
		CollisionInfo backup;

		int collCount = 0;
		for (int i = 0; i < count; ++i)
		{
			if (allCollInfo[i].bCollision)
			{
				++collCount;
				result.collNormal += (allCollInfo[i].collNormal * allCollInfo[i].fPenetration);

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
			result = backup;
	}

	return result;
}

CollisionInfo PhysicsScene::stitched2Plane(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return plane2Stitched(obj2, obj1);
}

CollisionInfo PhysicsScene::stitched2Sphere(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return sphere2Stitched(obj2, obj1);
}

CollisionInfo PhysicsScene::stitched2Box(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return box2Stitched(obj2, obj1);
}

CollisionInfo PhysicsScene::stitched2Poly(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return poly2Stitched(obj2, obj1);
}

CollisionInfo PhysicsScene::stitched2Stitched(PhysicsObject * obj1, PhysicsObject * obj2)
{
	Stitched* stitched1 = (Stitched*)obj1;
	Stitched* stitched2 = (Stitched*)obj2;

	vector<CollisionInfo> allCollInfo;
	CollisionInfo result;
	result.collNormal = { 0,0 };
	int count1 = stitched1->GetPolyCount();
	int count2 = stitched2->GetPolyCount();

	for (int i = 0; i < count1; ++i)
	{
		auto poly1 = stitched1->GetPoly(i);

		allCollInfo.push_back(poly2Stitched(poly1, stitched2));
		result.bCollision += allCollInfo[i].bCollision;

		//for (int j = 0; j < count2; ++j)
		//{
		//	auto poly2 = stitched2->GetPoly(j);
		//	allCollInfo.push_back(poly2Poly(poly1, poly2));
		//	result.bCollision += allCollInfo[i].bCollision;
		//}
	}

	if (result.bCollision)
	{
		CollisionInfo backup;

		int collCount = 0;
		for (int i = 0; i < allCollInfo.size(); ++i)
		{
			if (allCollInfo[i].bCollision)
			{
				++collCount;
				result.collNormal += (allCollInfo[i].collNormal * allCollInfo[i].fPenetration);

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
			result = backup;
	}

	return result;
}

void PhysicsScene::Restitution(float overlap, glm::vec2 const& collNormal, RigidBody * rb1, RigidBody * rb2)
{
	if (overlap <= 0.075f)
		return;

	float ratio = 1;
	if (rb2)
	{
		float rb1InvMass = 1 / rb1->getMass();
		float rb2InvMass = 1 / rb2->getMass();

		float rb1Mom = length(rb1->getVelocity()) * rb1InvMass;
		float rb2Mom = length(rb2->getVelocity()) * rb2InvMass;

		ratio = rb1Mom / (rb1Mom + rb2Mom);
		rb2->setPosition(rb2->getPosition() + (collNormal * overlap * (1 - ratio)));
		rb1->setPosition(rb1->getPosition() - (collNormal * overlap * ratio));
	}
	else
		rb1->setPosition(rb1->getPosition() + (collNormal * overlap * ratio));
	return;
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
	}

	debugCount++;
	if (debugCount > DEBUG_FREQ)
		debugCount = 0;
}

bool PhysicsScene::ProjectionOverlap(float const & min1, float const & max1, float const & min2, float const & max2, float & overlap)
{
	if (max1 <= min2)	return false;
	if (max2 <= min1)	return false;

	// Overlap
	float fMin = min(min1, min2);
	float fMax = max(max1, max2);

	overlap = fMax - fMin;
	return true;
}
