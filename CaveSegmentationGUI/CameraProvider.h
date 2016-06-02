#pragma once

#include <glm/glm.hpp>

class CameraProvider
{
public:
	virtual const glm::mat4& GetViewMatrix() = 0;
	virtual const glm::mat4& GetProjectionMatrix() = 0;
	virtual glm::mat4 GetViewRotationMatrix() = 0;
};