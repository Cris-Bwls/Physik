#include "CollisionTests.h"
#include"Plane.h"


int CollisionTests::TestPointTriangle(vec3 const & point, vec3 const & a, vec3 const & b, vec3 const & c)
{
	float u, v, w;
	Barycentric(point, a, b, c, u, v, w);

	return v >= 0.0f && w >= 0.0f && (v + w) <= 1.0f;
}

float CollisionTests::TriArea2D(vec2 const & a, vec2 const & b, vec2 const & c)
{
	return (a.x - b.x) * (b.y, c.y) - (b.x - c.x) * (a.y, b.y);
}

void CollisionTests::Barycentric(vec3 const & point, vec3 const & a, vec3 const & b, vec3 const & c, float & u, float & v, float & w)
{
	vec3 m = cross(b - a, c - a);
	float nu, nv, ood;
	float x = abs(m.x), y = abs(m.y), z = abs(m.z);

	if (x >= y && x >= z)
	{
		nu = TriArea2D({ point.y, point.z }, { b.y, b.z }, { c.y, c.z });
		nv = TriArea2D({ point.y, point.z }, { c.y, c.z }, { a.y, a.z });
		ood = 1.0f / m.x;
	}
	else if (y >= x && y >= z)
	{
		nu = TriArea2D({ point.x, point.z }, { b.x, b.z }, { c.x, c.z });
		nv = TriArea2D({ point.x, point.z }, { c.x, c.z }, { a.x, a.z });
		ood = 1.0f / m.y;
		//ood = 1.0f / -m.y; (IF THINGS DONT WORK TRY THIS)
	}
	else
	{
		nu = TriArea2D({ point.x, point.y }, { b.x, b.y }, { c.x, c.y });
		nv = TriArea2D({ point.x, point.y }, { c.x, c.y }, { a.x, a.y });
		ood = 1.0f / m.z;
	}

	u = nu * ood;
	v = nv * ood;
	w = 1.0f - u - v;
}

vec3 CollisionTests::ClosestPointOnPlane(vec3 const & point, Plane const & plane)
{
	vec3* pNormal = &plane.GetNormal();
	float t = dot(*pNormal, point) - plane.GetDistance();
	return point - t * (*pNormal);
}

float CollisionTests::DistPointPlane(vec3 const & point, Plane const & plane)
{
	vec3* pNormal = &plane.GetNormal();
	return (dot(*pNormal, point) - plane.GetDistance()) / dot(*pNormal, *pNormal);
}
