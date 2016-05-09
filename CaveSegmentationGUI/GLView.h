#pragma once

#include <QGLWidget>
#include <glm/glm.hpp>

class GLView : public QGLWidget
{
	Q_OBJECT
public:
	GLView(QWidget* parent);

signals:
	void inited(GLView* sender);

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

protected:
	double fovy;
	float znear, zfar;

	double pan, tilt;
	float focusLength;
	glm::vec3 focus;

	bool panningTilting;
	bool tracking;
	QPoint dragStart;
};

