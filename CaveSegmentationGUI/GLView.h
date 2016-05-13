#pragma once

#include <QOpenGLWidget>
#include <QOpenGLDebugLogger>
#include <glm/glm.hpp>

#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

class GLView : public QOpenGLWidget, public CameraProvider, public OpenGLContextProvider
{
	Q_OBJECT
public:
	GLView(QWidget* parent);

	const glm::mat4& GetViewMatrix();
	const glm::mat4& GetProjectionMatrix();
	void MakeOpenGLContextCurrent();

signals:
	void inited(GLView* sender);

protected:
	virtual void paintGL() = 0;
	virtual void initializeGL();
	virtual void resizeGL(int width, int height);
	void align_to_bounding_box(glm::vec3 min, glm::vec3 max);

	virtual void wheelEvent(QWheelEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent*);

	void recalculateProjection();
	void recalculateView();
	void recompileAll();

	void handleLoggedMessage(const QOpenGLDebugMessage&);

protected:
	double fovy;
	float znear, zfar;

	double pan, tilt;
	float focusLength;
	glm::vec3 focus;

	bool panningTilting;
	bool tracking;
	QPoint dragStart;

	glm::mat4 view, proj;

	QOpenGLDebugLogger glLogger;
};

