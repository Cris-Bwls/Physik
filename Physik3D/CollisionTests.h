#pragma once
#include <glm/ext.hpp>

using namespace glm;

class CollisionTests
{
public:
	static int TestPointTriangle(vec3 const& point, vec3 const& a, vec3 const& b, vec3 const& c);

private:
	inline CollisionTests() {};
	inline ~CollisionTests() {};

	static float TriArea2D(vec2 const& a, vec2 const& b, vec2 const& c);
	static void Barycentric(vec3 const& point, vec3 const& a, vec3 const& b, vec3 const& c, float & u, float & v, float & w);
};

