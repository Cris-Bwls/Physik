#include "PhysicsScene.h"
#include "PhysicsObject.h"


PhysicsScene::PhysicsScene()
{
	m_timeStep = 0.01f;
	m_gravity = { 0,0 };
}


PhysicsScene::~PhysicsScene()
{
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
			m_actors.erase(m_actors.begin + i);
			return true;
		}
	}

	return false;
}

void PhysicsScene::Update(float dt)
{
	static float accumulatedTime = 0.0f;
	accumulatedTime += dt;

	while (accumulatedTime >= m_timeStep)
	{
		for each (PhysicsObject* actor in m_actors)
		{
			actor->FixedUpdate(m_gravity, m_timeStep);
		}

		accumulatedTime -= m_timeStep;
	}
}

void PhysicsScene::UpdateGizmos()
{
	for each (PhysicsObject* actor in m_actors)
	{
		actor->makeGizmo();
	}
}
