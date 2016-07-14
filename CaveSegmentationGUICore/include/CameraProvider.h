#pragma once

#include "cavesegmentationguicore_global.h"

#include <glm/glm.hpp>

class CAVESEGMENTATIONGUICORE_EXPORT CameraProvider
{
public:
	virtual const glm::mat4& GetViewMatrix() = 0;
	virtual const glm::mat4& GetProjectionMatrix() = 0;
	virtual glm::mat4 GetViewRotationMatrix() = 0;
};