#include "PhysikApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include <Gizmos.h>
#include <glm/ext.hpp>

PhysikApp::PhysikApp() {

}

PhysikApp::~PhysikApp() {

}

bool PhysikApp::startup() {
	
	// increase the 2d line count to maximize the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);
	m_pPhysicsScene = new PhysicsScene();

	m_2dRenderer = new aie::Renderer2D();

	// TODO: remember to change this when redistributing a build!
	// the following path would be used instead: "./font/consolas.ttf"
	m_font = new aie::Font("./bin/font/consolas.ttf", 32);

	return true;
}

void PhysikApp::shutdown() {

	delete m_pPhysicsScene;

	delete m_font;
	delete m_2dRenderer;
}

void PhysikApp::update(float deltaTime) {

	// input example
	aie::Input* input = aie::Input::getInstance();

	aie::Gizmos::clear();
	m_pPhysicsScene->Update(deltaTime);
	m_pPhysicsScene->UpdateGizmos();

	// exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void PhysikApp::draw() {

	// wipe the screen to the background colour
	clearScreen();

	// begin drawing sprites
	m_2dRenderer->begin();

	// draw your stuff here!
	static float aspectRatio = 16 / 9.f;
	static float bottom = 100.0f / aspectRatio;
	aie::Gizmos::draw2D(glm::ortho<float>(-100, 100,
		-bottom, bottom, -1.0f, 1.0f));
	
	// output some text, uses the last used colour
	m_2dRenderer->drawText(m_font, "Press ESC to quit", 0, 0);

	// done drawing sprites
	m_2dRenderer->end();
}