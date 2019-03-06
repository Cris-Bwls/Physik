#include "Stitched.h"

Stitched::Stitched(vector<vector<vec2>> const & allVertices, vec2 position, vec2 velocity, float rotation, float fAngVel, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag, glm::vec4 colour) :
	RigidBody(ShapeID::Stitched, position, velocity, rotation, fAngVel, mass, elasticity, fFricCoStatic, fFricCoDynamic, fDrag, fAngDrag)
{
	m_Colour = colour;

	for (int i = 0; i < allVertices.size(); ++i)
	{
		vec2 pos = vec2(0.0f, 0.0f);
		for (int j = 0; j < allVertices[i].size(); ++j)
		{
			pos += allVertices[i][j];
		}
		pos /= (float)allVertices[i].size();
		m_PolyRelPos.push_back(pos);

		vector<vec2> verts;
		for (int j = 0; j < allVertices[i].size(); ++j)
		{
			verts.push_back(allVertices[i][j] - pos);
		}
		
		Poly* poly = new Poly(verts, position + pos, { 0,0 }, rotation, 1, 1, 1, 1, 1, 1, 1, colour);
		m_Polys.push_back(poly);
	}

	Transform pos = Transform();
	pos.SetPosition(m_position);

	Transform rot = Transform();
	rot.SetRotate2D(m_rotation);

	m_GlobalTransform.LocalTransform(pos.GetTransform(), rot.GetTransform(), Transform::Identity());
}

Stitched::~Stitched()
{
	for (int i = 0; i < m_Polys.size(); ++i)
	{
		delete m_Polys[i];
	}
}

void Stitched::fixedUpdate(vec2 const& gravity, float timeStep)
{
	RigidBody::fixedUpdate(gravity, timeStep);
	
	Transform pos = Transform();
	pos.SetPosition(m_position);

	Transform rot = Transform();
	rot.SetRotate2D(m_rotation);

	m_GlobalTransform.LocalTransform(pos.GetTransform(), rot.GetTransform(), Transform::Identity());
	
	for (int i = 0; i < m_Polys.size(); ++i)
	{
		Transform pos;
		pos.SetPosition(m_PolyRelPos[i]);

		m_Polys[i]->Move(m_GlobalTransform, pos);
	}
}


void Stitched::makeGizmo()
{
	for (int i = 0; i < m_Polys.size(); ++i)
	{
		m_Polys[i]->makeGizmo();
	}
}
