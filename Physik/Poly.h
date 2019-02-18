#pragma once
#include "RigidBody.h"
#include"Sphere.h"
#include <glm/ext.hpp>
#include <vector>

using namespace glm;
using std::vector;

class Poly : public RigidBody
{
public:
	Poly(vector<vec2> const& vertices, vec2 position, vec2 velocity, float rotation, float mass, float elasticity, glm::vec4 colour);
	~Poly();

	inline vector<vec2> GetVerts() const { return m_Vertices; }
	inline void SetVerts(vector<vec2> const& vertices) { m_Vertices = vertices; CreateBroadColl(); };

private:
	void CreateBroadColl();


	vec4 m_Colour;
	vector<vec2> m_Vertices;
	Sphere* m_pBroadColl;
};

