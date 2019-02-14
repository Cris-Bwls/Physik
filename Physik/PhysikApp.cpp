#include "PhysikApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include <Gizmos.h>
#include <glm/ext.hpp>
#include "Plane.h"
#include "Sphere.h"
#include "Box.h"

using namespace glm;

PhysikApp::PhysikApp() {

}

PhysikApp::~PhysikApp() {

}

bool PhysikApp::startup() {
	
	// increase the 2d line count to maximize the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);


	m_2dRenderer = new aie::Renderer2D();

	// TODO: remember to change this when redistributing a build!
	// the following path would be used instead: "./font/consolas.ttf"
	m_font = new aie::Font("./bin/font/consolas.ttf", 32);


	m_pPhysicsScene = new PhysicsScene();
	m_pPhysicsScene->setGravity(vec2(0, 0));
	m_pPhysicsScene->setTimeStep(0.01f);

	vec2 normal1 = { 1,0 };
	vec2 normal2 = { 0,1 };

	normal1 = normalize(normal1);
	normal2 = normalize(normal2);

	float fAspectRatio = getWindowWidth() / getWindowHeight();

	Plane* plane1 = new Plane(normal1, -90.0f);
	Plane* plane2 = new Plane(normal1, 90.0f);
	Plane* plane3 = new Plane(normal2, 56.0f);// 90.0f / fAspectRatio);
	Plane* plane4 = new Plane(normal2, -56.0f);//-90.0f / fAspectRatio);
	m_pPhysicsScene->AddActor(plane1);
	m_pPhysicsScene->AddActor(plane2);
	m_pPhysicsScene->AddActor(plane3);
	m_pPhysicsScene->AddActor(plane4);

	Box* box1 = new Box({ 5,5 }, { 20, 20 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	//Box* box2 = new Box({ 5,5 }, { 0, 0 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	//Box* box3 = new Box({ 5,5 }, { 30, 0 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	//Box* box4 = new Box({ 5,5 }, { 0, 30 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	//Box* box5 = new Box({ 5,5 }, { 15, 10 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	//Box* box6 = new Box({ 5,5 }, {10, 15 }, { 0,0 }, 1, 1.0f, { 0,0,1,1 }, true);
	m_pPhysicsScene->AddActor(box1);
	//m_pPhysicsScene->AddActor(box2);
	//m_pPhysicsScene->AddActor(box3);
	//m_pPhysicsScene->AddActor(box4);
	//m_pPhysicsScene->AddActor(box5);
	//m_pPhysicsScene->AddActor(box6);
	box1->applyForce({ 0,-20 });
	//box2->applyForce({ -5,0 });
	//box3->applyForce({ -5,0 });
	//box4->applyForce({ -5,0 });
	//box5->applyForce({ -5,0 });
	//box6->applyForce({ -5,0 });

	Sphere* ball1 = new Sphere(vec2(-20, 0), vec2(0, 0), 4.0f, 1.0f, 4, vec4(1, 0, 0, 1));
	Sphere* ball2 = new Sphere(vec2(0, -20), vec2(0, 0), 4.0f, 1.0f, 4, vec4(0, 1, 0, 1));
	
	m_pPhysicsScene->AddActor(ball1);
	m_pPhysicsScene->AddActor(ball2);
	ball1->applyForce(vec2(15, 0));
	ball2->applyForce(vec2(0, 10));
	
	for (int i = 0; i < 10; ++i)
	{
		Sphere* ball = new Sphere(10.0f * circularRand(5.0f), circularRand(10.0f), 3.0f, 1.0f, 4, vec4(linearRand(0.0f, 1.0f), linearRand(0.0f, 1.0f), linearRand(0.0f, 1.0f), 1));
		m_pPhysicsScene->AddActor(ball);
	
		//Box* box = new Box({ 5,5 }, 8.0f * circularRand(5.0f), {0,0}, 1, 1.0f, vec4(linearRand(0.0f, 1.0f), linearRand(0.0f, 1.0f), linearRand(0.0f, 1.0f), 1), true);
		//box->applyForce(circularRand(15.0f));
		//m_pPhysicsScene->AddActor(box);
	}

	return true;
}

void PhysikApp::shutdown() {

	delete m_font;
	delete m_2dRenderer;
	delete m_pPhysicsScene;

	aie::Gizmos::destroy();
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