#pragma once

#include <QGLWidget>
#include <glm/glm.hpp>
#include "Common.h"
#include <QTimer>

class GLView : public QGLWidget
{
	Q_OBJECT
public:
	GLView(QWidget* parent);
	~GLView();

	void setNearClip(float value);
	float* depthBuffer;

signals:
	void inited(GLView* sender);
	void viewUpdated();

protected:
	virtual void paintGL() = 0;
	virtual void initializeGL();
	virtual void resizeGL(int width, int height);
	void align_to_bounding_box(glm::vec3 min, glm::vec3 max);

	void wheelEvent(QWheelEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseMoveEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);

	void recalculateProjection();
	void recalculateView();
	void recompileAll();

	void issueViewChanged();
	void findPositionUnderMouse(int x, int y, glm::vec3&);
	void readDepthBuffer();

protected slots:
	void timer_timeout();

protected:
	double virtualScreenHeight;
	float znear, zfar;

	double pan, tilt;
	float focusLength, objectRadius;
	glm::vec3 focus, objectCenter;

	bool panningTilting;
	bool tracking;
	QPoint dragStart;

	float nearClip;

	QTimer timer;

	bool measuring;
	glm::vec3 measureStart, measureEnd;
	
	int depthBufferSize;
};

