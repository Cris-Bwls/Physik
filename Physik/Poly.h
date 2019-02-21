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
	inline void SetVerts(vector<vec2> const& vertices) { m_Vertices = vertices; CreateBroadColl(); CreateSNorms(); };
	inline int GetVerticeCount() const { return m_Vertices.size(); };
	inline int GetSNormCount() const { return m_SNorms.size(); };

	inline Sphere* GetBroadColl() const { return m_pBroadColl; };

	void fixedUpdate(vec2 const& gravity, float timeStep);
	void makeGizmo();

	vec2 GetRotatedVert(int index) const;
	vec2 GetRotatedSNorm(int index) const;

	void Project(vec2 const& axis, float & min, float & max);

	bool checkCollision(PhysicsObject* pOther) { return false; };

private:
	void CreateBroadColl();
	void CreateSNorms();

	vec4 m_Colour;
	vector<vec2> m_SNorms;
	vector<vec2> m_Vertices;
	Sphere* m_pBroadColl = nullptr;
};

