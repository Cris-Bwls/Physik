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
						CalcTorque(info.collPoint, (RigidBody*)object2);
						((Plane*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);
						((RigidBody*)object2)->resolveFriction(object1, info.collNormal, m_gravity, m_timeStep);

						// DEBUG
						((RigidBody*)object2)->InvertIsFilled();
					}
					else if (shapeID2 == (int)ShapeID::Plane)
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1);
						CalcTorque(info.collPoint, (RigidBody*)object1);
						((Plane*)object2)->resolveCollision((RigidBody*)object2, info.collNormal);
						((RigidBody*)object1)->resolveFriction(object2, info.collNormal, m_gravity, m_timeStep);

						// DEBUG
						((RigidBody*)object1)->InvertIsFilled();
					}
					else
					{
						Restitution(info.fPenetration, info.collNormal, (RigidBody*)object1, (RigidBody*)object2);
						CalcTorque(info.collPoint, (RigidBody*)object1, (RigidBody*)object2);
						((RigidBody*)object1)->resolveCollision((RigidBody*)object2, info.collNormal);
						((RigidBody*)object1)->resolveFriction(object2, info.collNormal, m_gravity, m_timeStep);

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
			result.collPoint = sphere2->getPosition() + collisionNormal * (sphere2->getRadius() - intersection);

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
		result.fPenetration = pen;
		result.collNormal = collNorm;
		result.collPoint = {0,0};
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
	
	float distance = FLT_MAX;
	vec2 closest = { 0,0 };

	for (int i = 0; i < poly2->GetVerticeCount(); ++i)
	{
		vec2 vert = poly2->GetRotatedVert(i) + poly2->getPosition();
		float tempDist = DistPointPlane(vert, plane1);
		if (tempDist < distance)
		{
			distance = tempDist;
			closest = ClosestPtPointPlane(vert, plane1);
		}
	}

	sat.collPoint = closest;

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
				result.collPoint += allCollInfo[i].collPoint;
				
				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;
		result.collPoint /= collCount;

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

			result.collPoint = sphere1->getPosition() + (result.collNormal * (sphere1->getRadius() - pen));

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

			result.collPoint = spherePos + (result.collNormal * (sphere1->getRadius() - pen));

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

	if (sat.bCollision)
		sat.collPoint = sphere1->getPosition() + (sat.collNormal * (sphere1->getRadius() - sat.fPenetration));

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
				result.collPoint += allCollInfo[i].collPoint;

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;
		result.collPoint /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
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
		result.collPoint = { 0,0 };
		
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

	if (sat.bCollision)
	{
		float distance = FLT_MAX;
		vec2 closest = {0,0};

		for (int i = 0; i < 4; ++i)
		{
			vec2 boxStart = boxVerts[i];
			vec2 boxEnd;
			if (i + 1 >= 4)
				boxEnd = boxVerts[0];
			else
				boxEnd = boxVerts[i + 1];

			for (int j = 0; j < poly2->GetVerticeCount(); ++j)
			{
				vec2 polyStart = poly2->getPosition() + poly2->GetRotatedVert(j);
				vec2 polyEnd = poly2->getPosition();
				if (j + 1 >= poly2->GetVerticeCount())
					polyEnd += poly2->GetRotatedVert(0);
				else
					polyEnd += poly2->GetRotatedVert(j+1);

				vec2 tempClose1, tempClose2;
				float s, t;

				float tempDist = ClosestPtSegmentSegment(boxStart, boxEnd, polyStart, polyEnd, s, t, tempClose1, tempClose2);
				if (tempDist < distance)
				{
					distance = tempDist;
					closest = (tempClose1 + tempClose2) * 0.5f;
				}
			}
		}

		sat.collPoint = closest;
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
				result.collPoint += allCollInfo[i].collPoint;

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;
		result.collPoint /= collCount;

		result.fPenetration = length(result.collNormal);
		if (result.fPenetration > FLT_EPSILON)
			result.collNormal = normalize(result.collNormal);
		else
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

	if (sat.bCollision)
	{
		float distance = FLT_MAX;
		vec2 closest;
		for (int i = 0; i < poly1->GetVerticeCount(); ++i)
		{
			vec2 poly1Start = poly1->getPosition() + poly1->GetRotatedVert(i);
			vec2 poly1End = poly1->getPosition();
			if (i + 1 >= poly1->GetVerticeCount())
				poly1End += poly1->GetRotatedVert(0);
			else
				poly1End += poly1->GetRotatedVert(i + 1);

			for (int j = 0; j < poly2->GetVerticeCount(); ++j)
			{
				vec2 poly2Start = poly2->getPosition() + poly2->GetRotatedVert(j);
				vec2 poly2End = poly2->getPosition();
				if (j + 1 >= poly2->GetVerticeCount())
					poly2End += poly2->GetRotatedVert(0);
				else
					poly2End += poly2->GetRotatedVert(j + 1);

				vec2 tempClose1, tempClose2;
				float s, t;

				float tempDist = ClosestPtSegmentSegment(poly1Start, poly1End, poly2Start, poly2End, s, t, tempClose1, tempClose2);
				if (tempDist < distance)
				{
					distance = tempDist;
					closest = (tempClose1 + tempClose2) * 0.5f;
				}
			}
		}

		sat.collPoint = closest;
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
				result.collPoint += allCollInfo[i].collPoint;

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;
		result.collPoint /= collCount;

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
				result.collPoint += allCollInfo[i].collPoint;

				if (allCollInfo[i].fPenetration > backup.fPenetration)
				{
					backup = allCollInfo[i];
				}
			}
		}
		result.collNormal /= collCount;
		result.collPoint /= collCount;

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
	//if (overlap <= 0.075f)
	//	return;

	bool debug = false;
	bool rb1GoodVel = true;
	bool rb2GoodVel = true;

	float ratio = 1;

	bool BROKEN = true;
	if (BROKEN)
	{
		if (rb2)
		{
			float rb1InvMass = 1 / rb1->getMass();
			float rb2InvMass = 1 / rb2->getMass();

			float rb1Mom = length(rb1->getVelocity()) * rb1InvMass;
			float rb2Mom = length(rb2->getVelocity()) * rb2InvMass;

			ratio = rb1Mom / (rb1Mom + rb2Mom);
			rb2->setPosition(rb2->getPosition() + (collNormal * overlap * (1 - ratio)));
		}
		rb1->setPosition(rb1->getPosition() - (collNormal * overlap * ratio));
		return;
	}


	// check velocity is real
	vec2 rb1Pos = rb1->getPosition();
	vec2 rb1Vel = rb1->getVelocity();
	vec2 rb1UnitVel = normalize(rb1Vel);
	float rb1Speed = length(rb1Vel);

	// NAN check
	if (rb1UnitVel.x != rb1UnitVel.x || rb1UnitVel.y != rb1UnitVel.y)
	{
		rb1GoodVel = false;
		rb1Vel = { 0,0 };
		rb1UnitVel = { 0,0 };
		rb1Speed = 0;
	}
	vec2 relVel = rb1UnitVel;

	/// These are for debugging
	vec2 rb1Offset = { 0,0 };
	vec2 rb2Offset = { 0,0 };

	
	// IF rb2 exists
	if (rb2)
	{
		// check velocity is real
		vec2 rb2Pos = rb2->getPosition();
		vec2 rb2Vel = rb2->getVelocity();
		vec2 rb2UnitVel = normalize(rb2Vel);
		float rb2Speed = length(rb2Vel);

		// NAN check
		if (rb2UnitVel.x != rb2UnitVel.x)
		{
			rb2GoodVel = false;
			rb2Vel = { 0,0 };
			rb2UnitVel = { 0,0 };
			rb2Speed = 0;
		}

		relVel -= rb2UnitVel;

		// Get the real ratio
		float rb1Mom = rb1Speed * (1 / rb1->getMass());
		float rb2Mom = rb2Speed * (1 / rb2->getMass());

		ratio = rb1Mom / (rb1Mom + rb2Mom);

		if (ratio != ratio)
			return;

		if (length(relVel) < FLT_EPSILON || abs(dot(relVel, collNormal)) < 0.1f)
		{
			relVel = collNormal;
		}
	
		// Restitute rb2
		rb2Offset = relVel * overlap * (1 - ratio);
		rb2->setPosition(rb2Pos - rb2Offset);
	}

	float test = abs(dot(relVel, collNormal));
	if (length(relVel) < FLT_EPSILON || test < 0.1f)
	{
		relVel = collNormal;
	}

	// Restitute rb1
	rb1Offset = relVel * overlap * ratio;
	rb1->setPosition(rb1Pos - rb1Offset);

	if (rb1->getPosition().x != rb1->getPosition().x)
		printf("FUCK");
	if (rb2)
		if (rb2->getPosition().x != rb2->getPosition().x)
			printf("FUCK");
}

void PhysicsScene::CalcTorque(glm::vec2 const& collPoint, RigidBody* rb1, RigidBody* rb2)
{

	vec2 rb1Pos = rb1->getPosition();
	vec2 rb1Vel = rb1->getVelocity();
	float rb1Radius = length(collPoint - rb1Pos);

	vec2 relVel = rb1Vel;
	float ratio = 1;

	if (rb2)
	{
		if (rb2->getShapeID() == ShapeID::Box && rb1->getShapeID() == ShapeID::Box)
		{
			return;
		}
		else if (rb1->getShapeID() == ShapeID::Box)
		{
			ratio = 0;
		}
		else 
		{
			float rb1InvMass = 1 / rb1->getMass();
			float rb2InvMass = 1 / rb2->getMass();

			ratio = rb1InvMass / (rb1InvMass + rb2InvMass);
		}

		if ((rb2->getShapeID() != ShapeID::Box))
		{
			vec2 rb2Pos = rb2->getPosition();
			vec2 rb2Vel = rb2->getVelocity();
			float rb2Radius = length(collPoint - rb2Pos);

			relVel -= rb2Vel;

			float rb2RadToForce = dot(normalize(relVel), normalize(collPoint - rb2Pos));
			float rb2Theta = acosf(rb2RadToForce);

			float rb2Torque = sinf(rb2Theta) * rb2Radius * length(relVel) * (1 - ratio);
			float rb2Inertia = 0.4 * rb2->getMass() * (rb2Radius * rb2Radius);

			float rb2AngVel = rb2->getAngularVelocity();
			rb2AngVel += rb2Torque / rb2Inertia * m_timeStep;

			if (rb2AngVel != rb2AngVel)
				printf("ANGULAR FUCK");
			rb2->setAngularVelocity(rb2AngVel);
		}
	}

	// EARLY EXIT
	if (rb1->getShapeID() == ShapeID::Box)
		return;

	float rb1RadToForce = dot(normalize(relVel), normalize(collPoint - rb1Pos));
	float rb1Theta = acosf(rb1RadToForce);

	float rb1Torque = sinf(rb1Theta) * rb1Radius * length(relVel) * ratio;
	float rb1Inertia = 0.4 * rb1->getMass() * (rb1Radius * rb1Radius);

	float rb1AngVel = rb1->getAngularVelocity();
	rb1AngVel += rb1Torque / rb1Inertia * m_timeStep;

	if (rb1AngVel != rb1AngVel)
		printf("ANGULAR FUCK");
	rb1->setAngularVelocity(rb1AngVel);

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

float PhysicsScene::DistPointPlane(glm::vec2 const & point, Plane * plane)
{
	return dot(plane->getNormal(), point) - plane->getDistance();
}

glm::vec2 PhysicsScene::ClosestPtPointPlane(glm::vec2 const & point, Plane * plane)
{
	float t = DistPointPlane(point, plane);
	return point - t * plane->getNormal();
}

void PhysicsScene::ClosestPtPointSegment(glm::vec2 const & point, glm::vec2 const & start, glm::vec2 end, float & t, glm::vec2 & closest)
{
	vec2 line = end - start;

	t = dot(point - start, line) / dot(line, line);

	t = Clamp(t, 0, 1);

	closest = start + t * line;
}

float PhysicsScene::ClosestPtSegmentSegment(glm::vec2 const & start1, glm::vec2 const & end1, glm::vec2 const & start2, glm::vec2 const & end2, 
											float & s, float & t, glm::vec2 & closest1, glm::vec2 & closest2)
{
	vec2 d1 = end1 - start1;
	vec2 d2 = end2 - start2;
	vec2 r = start1 - start2;

	float a = dot(d1, d1);
	float e = dot(d2, d2);
	float f = dot(d2, r);

	if (a <= FLT_EPSILON && e <= FLT_EPSILON)
	{
		s = t = 0.0f;
		closest1 = start1;
		closest2 = start2;

		return dot(closest1 - closest2, closest1 - closest2);
	}
	if (a <= FLT_EPSILON)
	{
		s = 0.0f;
		t = f / e;
		t = Clamp(t, 0, 1);
	} 
	else
	{
		float c = dot(d1, r);
		if (e <= FLT_EPSILON)
		{
			t = 0.0f;
			s = Clamp(-c / a, 0, 1);
		}
		else
		{
			float b = dot(d1, d2);
			float denom = a * e - b * b;

			if (denom != 0.0f)
			{
				s = Clamp((b*f - c * e) / denom, 0, 1);
			}
			else
			{
				s = 0.0f;
			}
			t = (b*s + f) / e;

			if (t < 0.0f)
			{
				t = 0.0f;
				s = Clamp(-c / a, 0, 1);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = Clamp((b - c) / a, 0, 1);
			}
		}
	}

	closest1 = start1 + d1 * s;
	closest2 = start2 + d2 * t;
	return dot(closest1 - closest2, closest1 - closest2);
}

float PhysicsScene::Clamp(float n, float min, float max)
{
	if (n < min) return min;
	if (n > max) return max;
	return n;
}
