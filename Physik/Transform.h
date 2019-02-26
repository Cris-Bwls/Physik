#pragma once
#include <glm\ext.hpp>


class Transform
{
public:
	Transform(glm::mat3* transform = nullptr);
	~Transform();

	inline glm::mat3 GetTransform() const& { return m_Transform; }
	glm::vec2 GetPosition() const&;
	
	void LocalTransform(glm::mat3 const& posMat, glm::mat3 const& rotMat, glm::mat3 const& scaleMat);
	void GlobalTransform(glm::mat3 const& global, glm::mat3 const& local);
	void SetRotate2D(float fRadians);
	void SetPosition(glm::vec2 const& pos);
	void SetScale(glm::vec2 const& scale);

	static glm::mat3 Identity();

private:
	glm::mat3 m_Transform;
};

