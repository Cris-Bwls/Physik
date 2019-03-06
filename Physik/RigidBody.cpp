#include "RigidBody.h"
#include <iostream>
#include <Gizmos.h>

using namespace glm;

RigidBody::RigidBody(ShapeID shapeID, glm::vec2 position, glm::vec2 velocity, float rotation, float fAngVelocity,  float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag) 
	: PhysicsObject::PhysicsObject(shapeID, fFricCoStatic, fFricCoDynamic)
{
	m_position = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_angularVelocity = fAngVelocity;
	m_mass = mass;
	m_elasticity = elasticity;
	m_drag = fDrag;
	m_angularDrag = fAngDrag;

	m_ResolutionForceSum = { 0,0 };
}

RigidBody::~RigidBody()
{
}

void RigidBody::fixedUpdate(vec2 const& gravity, float timeStep)
{
	if (m_position.x != m_position.x)
	{
		printf(" FUCK");
	}

	applyForce(gravity * m_mass * timeStep);
	ApplyDrags(timeStep);

	m_position += m_velocity * timeStep;
	m_rotation += m_angularVelocity * timeStep;
}

void RigidBody::debug()
{
	printf(" ID %i ", (int)m_ShapeId);
	printf(" POS x %f, y %f ", m_position.x, m_position.y);
	printf(" VEL x %f, y %f ", m_velocity.x, m_velocity.y);	
	printf(" ROT %f ", m_rotation);
	printf(" ANG VEL %f ", m_angularVelocity);
}

void RigidBody::applyForce(vec2 const& force)
{
	m_velocity += force / m_mass;
}

void RigidBody::applyForceToActor(RigidBody* actor2, vec2 const& force)
{
	applyForce(force);
	actor2->applyForce(-force);
}

void RigidBody::ApplyDrags(float const& timeStep)
{
	m_velocity -= m_velocity * m_drag * timeStep;
	m_angularVelocity -= m_angularVelocity * m_angularDrag * timeStep;
}

void RigidBody::DebugVelocity(vec2 const& startPoint)
{
	vec2 endPoint = startPoint + (m_velocity * 0.5f);
	aie::Gizmos::add2DLine(startPoint, endPoint, { 1,1,1,1 });
}

void RigidBody::resolveCollision(RigidBody* actor2, vec2 const& normal)
{
	vec2 relativeVelocity = actor2->getVelocity() - m_velocity;
	float elasticity = (actor2->getElasticity() + m_elasticity) / 2;

	float j = dot(-(1 + elasticity) * relativeVelocity, normal) /
				dot(normal, normal * ((1 / m_mass + (1 / actor2->getMass()))));

	vec2 force = normal * j;
	if (force.x != force.x)
		printf("FUCK");

	applyForceToActor(actor2, -force);
}

void RigidBody::resolveFriction(PhysicsObject* obj2, glm::vec2 const& normal, glm::vec2 const& gravity, float const timeStep)
{
	// DEBUG
//CURRENTLY BROKEN
	if (true)
		return;

	RigidBody* rb2 = nullptr;
	if (obj2->getShapeID() != ShapeID::Plane)
		rb2 = (RigidBody*)obj2;

	vec2 relVel = m_velocity;
	if (rb2)
	{
		relVel -= rb2->getVelocity();
	}

	// EARLY EXIT
	//if (abs(dot(normal, relVel)) >= FLT_EPSILON)
	//	return;

	float avgStaticCo = (m_fFricCoStatic + obj2->GetStaticFricCo()) / 2 * timeStep;
	float avgKineticCo = (m_fFricCoKinetic + obj2->GetKineticFricCo()) / 2 * timeStep;

	vec2 fricNormal;
	fricNormal.x = normal.y;
	fricNormal.y = -normal.x;

	float ratio = 1;
	if (rb2)
	{
		vec2 rb2Vel = rb2->getVelocity();
		float rb2VelAlongFric = dot(rb2Vel, fricNormal);
		float thisVelAlongFric = dot(m_velocity, fricNormal);

		ratio = abs(thisVelAlongFric) / (abs(rb2VelAlongFric) + abs(thisVelAlongFric));
		if (ratio != ratio)
			ratio = 1.0f;
	}

	float relVelAlongFric = dot(relVel, fricNormal);

	float baseFric = abs(dot(normal, gravity));

	float maxStaticFric = baseFric * avgStaticCo;
	float kineticFric = baseFric * avgKineticCo;
	if (relVelAlongFric < 0)
	{
		kineticFric = -kineticFric;
	}

	bool staticCheck = abs(relVelAlongFric) <= maxStaticFric;
	bool kineticCheck = abs(relVelAlongFric) <= abs(kineticFric);

	if (staticCheck || kineticCheck)
	{
		// Could not overcome static friction
		m_velocity = normal * dot(normal, m_velocity);
		if (rb2)
			m_velocity += fricNormal * dot(rb2->getVelocity(), fricNormal) * ratio;
		
		else if (rb2)
		{
			vec2 rb2Vel = normal * dot(normal, rb2->getVelocity());
			rb2Vel += fricNormal * dot(m_velocity, fricNormal) * (1- ratio);
			rb2->setVelocity(rb2Vel);

			if (rb2->getVelocity().y != rb2->getVelocity().y)
				printf(" FUCK ");
		}
		if (m_velocity.y != m_velocity.y)
			printf(" FUCK ");
		return;
	}


	
	// Apply kinetic friction

	//short sign = 1;
	//if (relVelAlongFric - dot(m_velocity, fricNormal) < relVelAlongFric)
	//	sign = -1;
	//
	//m_velocity += float(sign) * fricNormal * kineticFric * ratio;
	//
	//if (rb2)
	//{
	//	sign = 1;
	//	if (relVelAlongFric - dot(rb2->getVelocity(), fricNormal) < relVelAlongFric)
	//		sign = -1;
	//
	//	vec2 newVel = rb2->getVelocity();
	//	newVel += float(sign) * fricNormal * kineticFric * (1 - ratio);
	//
	//	rb2->setVelocity(newVel);
	//}

	float after = relVelAlongFric - kineticFric;
	if (abs(after) < 0)
		after = 0;

	float rb1VelAlongFric = dot(m_velocity, fricNormal);
	float rb1Fric = rb1VelAlongFric - (after * ratio);
	m_velocity -= rb1Fric * fricNormal;

	if (rb2)
	{
		vec2 rb2Vel = rb2->getVelocity();
		float rb2VelAlongFric = dot(rb2Vel, fricNormal);
		float rb2Fric = rb2VelAlongFric - (after * (1 - ratio));
		rb2Vel -= rb2Fric * fricNormal;

		rb2->setVelocity(rb2Vel);

		if (rb2->getVelocity().y != rb2->getVelocity().y)
			printf(" FUCK ");
	}

	if (m_velocity.y != m_velocity.y)
		printf(" FUCK ");
	
}
