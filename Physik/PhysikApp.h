#pragma once

#include "Application.h"
#include "Renderer2D.h"
#include "PhysicsScene.h"

class PhysikApp : public aie::Application {
public:

	PhysikApp();
	virtual ~PhysikApp();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

protected:

	PhysicsScene* m_pPhysicsScene;

	aie::Renderer2D*	m_2dRenderer;
	aie::Font*			m_font;
};