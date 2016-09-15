#pragma once

#include "cavesegmentationguicore_global.h"

#include <QOpenGLWidget>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QTimer>
#include <glm/glm.hpp>
#include <memory>

#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

class CAVESEGMENTATIONGUICORE_EXPORT HoldCameraEvents;

class CAVESEGMENTATIONGUICORE_EXPORT GLView : public QOpenGLWidget, public CameraProvider, public OpenGLContextProvider
{
	Q_OBJECT
public:
	GLView(QWidget* parent, bool automaticDepthBufferRetrieval = false, float eyeOffset = 0.0f, GLView* masterCam = nullptr);

	~GLView();

	void setCameraControlModifier(Qt::KeyboardModifier);

	const glm::mat4& GetViewMatrix();
	glm::mat4 GetViewRotationMatrix();
	const glm::mat4& GetProjectionMatrix();
	void MakeOpenGLContextCurrent();

	void setVirtualAspectMultiplier(float multiplier);

	//Requires prior call of readDepthBuffer()
	void findPositionUnderMouse(int x, int y, glm::vec3&);
	void readDepthBuffer();

	const float* depthBuffer() const;

	bool isAutomaticDepthBufferRetrievalEnabled() const;
	void enableAutomaticDepthBufferRetrieval();
	void disableAutomaticDepthBufferRetrieval();

	float getZNear() const;
	float getZFar() const;
	glm::vec3 getEye() const;

	void setFrontViewCutoff(float);

protected slots:
	void issueRepaint();

	void setCameraParameters(float pan, float tilt, const glm::vec3& focus, float focusLength);
	void setZRange(float znear, float zfar, float zeroParallaxInterpoll, float cursorDepthInterpol);

	void stillViewTimer_timeout();

signals:
	void inited(GLView* sender);
	void CameraChanged();
	void CameraStill(); //is emitted when the camera is standing still for a short time

	void camParamsChanged(float pan, float tilt, const glm::vec3& focus, float focusLength);
	void zRangeChanged(float znear, float zfar, float zeroParallaxInterpol, float cursorDepthInterpol);

protected:
	void emitCameraChanged();
	void emitCamParamsChanged();
	void emitZRangeChanged();

	virtual void paintGL() = 0;
	virtual void initializeGL();
	virtual void resizeGL(int width, int height);
	void align_to_bounding_box(glm::vec3 min, glm::vec3 max);

	void zoom(float amount);

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
	bool zooming;
	QPoint dragStart;

	glm::mat4 view, proj;

	QOpenGLDebugLogger glLogger;
	bool isPrimary;

	float eyeOffset;
	float cursorOffset, cursorDepth;

	float virtualAspectMultiplier;

	std::unique_ptr<QOpenGLShaderProgram> skyProgram;
	QOpenGLVertexArrayObject emptyVAO;

	float* _depthBuffer;
	int depthBufferSize;

	QTimer stillViewTimer;
	bool _enableAutomaticDepthBufferRetrieval;

	Qt::KeyboardModifier cameraControlModifier;
	Qt::KeyboardModifier cameraZoomModifier;

	float frontViewCutoff;

	HoldCameraEvents* cameraHold;
	friend class HoldCameraEvents;
};

//When active, delays all camera-related events of the GLView until destruction.
class CAVESEGMENTATIONGUICORE_EXPORT HoldCameraEvents
{
public:
	HoldCameraEvents(GLView* view);
	~HoldCameraEvents();

private:
	GLView* view;

	bool cameraChanged;
	bool camParamsChanged;
	bool zRangeChanged;

	friend class GLView;
};