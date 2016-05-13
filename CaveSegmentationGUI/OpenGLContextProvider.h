#pragma once

#include <QOpenGLContext>

class OpenGLContextProvider
{
public:
	virtual void MakeOpenGLContextCurrent() = 0;
};