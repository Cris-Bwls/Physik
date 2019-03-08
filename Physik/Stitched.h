#pragma once
#include "RigidBody.h"
#include "Poly.h"
#include <vector>
#include "Transform.h"

using std::vector;

class Stitched :
	public RigidBody
{
public:
	Stitched(vector<vector<vec2>> const& allVertices, vec2 position, vec2 velocity, float rotation, float fAngVel, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag, glm::vec4 colour);
	~Stitched();

	void fixedUpdate(vec2 const& gravity, float timeStep);
	void makeGizmo();

	inline int GetPolyCount() const& { return (int)m_Polys.size(); };
	inline Poly* GetPoly(int index) const { return m_Polys[index]; };

private:
	Transform m_GlobalTransform;

	vector<vec2> m_PolyRelPos;
	vector<Poly*> m_Polys;
	vec4 m_Colour;
};

