#pragma once

#include "cavesegmentationguicore_global.h"

#include <QOpenGLWidget>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <glm/glm.hpp>
#include <memory>

#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

class CAVESEGMENTATIONGUICORE_EXPORT GLView : public QOpenGLWidget, public CameraProvider, public OpenGLContextProvider
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

	//Requires prior call of readDepthBuffer()
	void findPositionUnderMouse(int x, int y, glm::vec3&);
	void readDepthBuffer();

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

	void renderSky();

	void recalculateProjection();
	void recalculateView();	

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

	std::unique_ptr<QOpenGLShaderProgram> skyProgram;
	QOpenGLVertexArrayObject emptyVAO;

	float* depthBuffer;
};

