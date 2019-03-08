#pragma once
#include "RigidBody.h"
#include"Sphere.h"
#include <glm/ext.hpp>
#include <vector>
#include "Transform.h"

using namespace glm;
using std::vector;

struct SurfaceNorm
{
	vec2 norm = {0,0};
	bool hasParallel = false;
};

class Poly : public RigidBody
{
public:
	Poly(vector<vec2> const& vertices, vec2 position, vec2 velocity, float rotation, float fAngVel, float mass, float elasticity, float fFricCoStatic, float fFricCoDynamic, float fDrag, float fAngDrag, glm::vec4 colour);
	~Poly();

	inline void SetRotation(float rotation) { m_rotation = rotation; };

	inline vector<vec2> GetVerts() const { return m_Vertices; }
	inline void SetVerts(vector<vec2> const& vertices) { m_Vertices = vertices; CreateBroadColl(); CreateSNorms(); };
	inline int GetVerticeCount() const { return (int)m_Vertices.size(); };
	inline int GetSNormCount() const { return (int)m_SNorms.size(); };
	inline bool GetSNormParallel(int index) const { return m_SNorms[index].hasParallel; }

	inline Sphere* GetBroadColl() const { return m_pBroadColl; };

	void fixedUpdate(vec2 const& gravity, float timeStep);
	void makeGizmo();
	void Move(Transform const& parentTransform, Transform const& localTransform);

	vec2 GetRotatedVert(int index) const;
	vec2 GetRotatedSNorm(int index) const;

	void Project(vec2 const& axis, float & min, float & max);

	bool checkCollision(PhysicsObject* pOther) { return false; };

private:
	void CreateBroadColl();
	void CreateSNorms();

	Transform m_GlobalTransform;

	vec4 m_Colour;
	vector<SurfaceNorm> m_SNorms;
	vector<vec2> m_Vertices;
	Sphere* m_pBroadColl = nullptr;
};

