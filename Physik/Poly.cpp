#include "Poly.h"
#include <Gizmos.h>

#define DEBUG true
#define SHOW_NORMALS false

Poly::Poly(vector<vec2> const & vertices, vec2 position, vec2 velocity, float rotation, float mass, float elasticity, glm::vec4 colour) :
	RigidBody::RigidBody(ShapeID::Poly, position, velocity, rotation, mass, elasticity)
{
	m_Colour = colour;
	m_Vertices = vertices;

	CreateSNorms();
	CreateBroadColl();
}

Poly::~Poly()
{
}

void Poly::fixedUpdate(vec2 const& gravity, float timeStep)
{
	RigidBody::fixedUpdate(gravity, timeStep);

	m_pBroadColl->setPosition(m_position);
}


void Poly::makeGizmo()
{
	if (DEBUG)
		m_pBroadColl->makeGizmo();

	vec2 start;
	vec2 end;
	uint count = m_Vertices.size();
	for (uint i = 0; i < count; ++i)
	{
		uint j = i + 1;
		if (j >= count)
			j = 0;

		start = GetRotatedVert(i) + m_position;
		end = GetRotatedVert(j) + m_position;
		if (m_bIsFilled)
			aie::Gizmos::add2DTri(start, m_position, end, m_Colour);
		else
			aie::Gizmos::add2DLine(start, end, m_Colour);
	}

	if (SHOW_NORMALS)
	{
		uint normalCount = m_SNorms.size();
		for (uint i = 0; i < normalCount; ++i)
		{
			uint j = i + 1;
			if (j >= count)
				j = 0;

			start = GetRotatedVert(i);
			end = GetRotatedVert(j);

			vec2 mid = ((start + end) * 0.5f) + m_position;

			aie::Gizmos::add2DLine(mid, m_SNorms[i] + mid, {1, 0, 0, 1});
		}
	}
}

vec2 Poly::GetRotatedVert(int index) const
{
	if (index >= m_Vertices.size())
		assert(!"m_Vertices index OUT OF BOUNDS");

	mat2 rotMat;
	rotMat[0][0] = cosf(m_rotation);
	rotMat[0][1] = sinf(m_rotation);
	rotMat[1][0] = -sinf(m_rotation);
	rotMat[1][1] = cosf(m_rotation);

	vec2 result = rotMat * m_Vertices[index];

	return result;
}

vec2 Poly::GetRotatedSNorm(int index) const
{
	if (index >= m_SNorms.size())
		assert(!"m_SNorms index OUT OF BOUNDS");

	mat2 rotMat;
	rotMat[0][0] = cosf(m_rotation);
	rotMat[0][1] = sinf(m_rotation);
	rotMat[1][0] = -sinf(m_rotation);
	rotMat[1][1] = cosf(m_rotation);

	vec2 result = rotMat * m_SNorms[index];

	return result;
}

void Poly::Project(vec2 const & axis, float & min, float & max)
{
	min = dot(axis, GetRotatedVert(0) + m_position);
	max = min;

	for (int i = 1; i < GetVerticeCount(); ++i)
	{
		float temp = dot(axis, GetRotatedVert(i) + m_position);
		if (temp < min)
		{
			min = temp;
		}
		else if (temp > max)
		{
			max = temp;
		}
	}
}

void Poly::CreateBroadColl()
{
	float radius = 0;
	for each (vec2 vert in m_Vertices)
	{
		float temp = length(vert);
		if (radius < temp)
			radius = temp;
	}
	radius += 0.1f;
	
	if (m_pBroadColl)
		delete m_pBroadColl;
	
	vec4 colour = { 1,1,1,1 };
	colour -= m_Colour;
	colour.a = 0.5f;

	m_pBroadColl = new Sphere(m_position, { 0,0 }, m_mass, m_elasticity, radius, colour);
}

void Poly::CreateSNorms()
{
	m_SNorms.clear();

	for (int i = 0; i < GetVerticeCount(); ++i)
	{
		int j = i + 1;
		if (j >= GetVerticeCount())
			j = 0;

		vec2 vec = m_Vertices[i] - m_Vertices[j];

		vec2 sNormal;
		sNormal.x = vec.y;
		sNormal.y = -vec.x;

		sNormal = normalize(sNormal);

		m_SNorms.push_back(sNormal);
	}

	// Parallel check
	for (int i = 0; i < (m_SNorms.size() - 1); ++i)
	{
		for (int j = i + 1; j < m_SNorms.size();)
		{
			vec2 lhs = m_SNorms[i];
			vec2 rhs = m_SNorms[j];

			float check = dot(lhs, rhs);

			if (abs(check) >= 1 - FLT_EPSILON)
			{
				m_SNorms.erase(m_SNorms.begin() + j);
			}
			else
			{
				++j;
			}
		}
	}
}
