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
	GLView(QWidget* parent, float eyeOffset = 0.0f, GLView* masterCam = nullptr);

	~GLView();

	const glm::mat4& GetViewMatrix();
	glm::mat4 GetViewRotationMatrix();
	const glm::mat4& GetProjectionMatrix();
	void MakeOpenGLContextCurrent();

	void setVirtualAspectMultiplier(float multiplier);

protected slots:
	void issueRepaint();

	void setCameraParameters(float pan, float tilt, const glm::vec3& focus, float focusLength);
	void setZRange(float znear, float zfar, float zeroParallaxInterpoll, float cursorDepthInterpol);

signals:
	void inited(GLView* sender);
	void CameraChanged();

	void camParamsChanged(float pan, float tilt, const glm::vec3& focus, float focusLength);
	void zRangeChanged(float znear, float zfar, float zeroParallaxInterpol, float cursorDepthInterpol);

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
	float zeroParallaxInterpol; //Interpolation between znear and zfar
	float cursorDepthInterpol; //Interpolation between znear and focus

	double pan, tilt;
	float focusLength;
	glm::vec3 focus;

	bool panningTilting;
	bool tracking;
	QPoint dragStart;

	glm::mat4 view, proj;

	QOpenGLDebugLogger glLogger;
	bool isPrimary;

	float eyeOffset;
	float cursorOffset, cursorDepth;

	float virtualAspectMultiplier;
};

