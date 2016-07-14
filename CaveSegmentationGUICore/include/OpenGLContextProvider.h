#pragma once

#include "cavesegmentationguicore_global.h"

#include <QOpenGLContext>

class CAVESEGMENTATIONGUICORE_EXPORT OpenGLContextProvider
{
public:
	virtual void MakeOpenGLContextCurrent() = 0;
};