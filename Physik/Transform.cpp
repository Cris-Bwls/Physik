#include "Transform.h"



Transform::Transform(glm::mat3* pTransform)
{
	if (pTransform)
		m_Transform = *pTransform;
	else
		m_Transform = Identity();
}

Transform::~Transform()
{
}

glm::vec2 Transform::GetPosition() const &
{
	glm::vec2 pos;
	
	pos[0] = m_Transform[2][0];
	pos[1] = m_Transform[2][1];

	return pos;
}

void Transform::LocalTransform(glm::mat3 const & posMat, glm::mat3 const & rotMat, glm::mat3 const & scaleMat)
{
	m_Transform = posMat * rotMat * scaleMat;
}

void Transform::GlobalTransform(glm::mat3 const & global, glm::mat3 const & local)
{
	m_Transform = global * local;
}

void Transform::SetRotate2D(float fRadians)
{
	m_Transform[0][0] = cosf(fRadians);
	m_Transform[0][1] = sinf(fRadians);

	m_Transform[1][0] = -sinf(fRadians);
	m_Transform[1][1] = cosf(fRadians);
}

void Transform::SetPosition(glm::vec2 const & pos)
{
	m_Transform[2][0] = pos.x;
	m_Transform[2][1] = pos.y;
}

void Transform::SetScale(glm::vec2 const & scale)
{
	m_Transform[0][0] = scale.x;
	m_Transform[1][1] = scale.y;
}

glm::mat3 Transform::Identity()
{
	glm::mat3 identity = glm::mat3(0);

	identity[0][0] = 1.0f;
	identity[1][1] = 1.0f;
	identity[2][2] = 1.0f;

	return identity;
}
