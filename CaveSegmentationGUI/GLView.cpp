#include "stdafx.h"

#include "GLView.h"


#include "GLView.h"
#include <gl\GLU.h>
#include <QWheelEvent>
#include <glm/gtc/type_ptr.hpp>

GLView::GLView(QWidget* parent) : QGLWidget(parent)
{
	fovy = 45;
	znear = 1;
	zfar = 10;
	focusLength = 5;
	focus = glm::vec3(0);

	pan = 0;
	tilt = 0;

	panningTilting = false;
	tracking = false;
}

void GLView::initializeGL()
{
	glClearColor(0, 0.1f, 0.3f, 0);

	glEnable(GL_PROGRAM_POINT_SIZE);

	glEnable(GL_DEPTH_TEST);

	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };

	GLfloat light_color[] = { 1, 1, 1, 1.0 };
	GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
	GLfloat light_direction1[] = { 1.0, -1.0, -1.0, 0.0 };
	GLfloat light_direction2[] = { -1.0, -1.0, 1.0, 0.0 };

	glShadeModel(GL_SMOOTH);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_direction1);
	glLightfv(GL_LIGHT1, GL_POSITION, light_direction2);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_color);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_color);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	recalculateView();

	emit inited(this);
}

void GLView::recalculateView()
{
	glm::vec3 dir(sin(tilt) * cos(pan), sin(pan), cos(tilt) * cos(pan));
	glm::vec3 eye = focus + focusLength * dir;
	glLoadIdentity();
	gluLookAt(eye.x, eye.y, eye.z, focus.x, focus.y, focus.z, 0, 1, 0);
	repaint();
}

void GLView::wheelEvent(QWheelEvent* e)
{
	auto factor = pow(0.999, e->delta());
	auto oldLength = focusLength;
	focusLength *= factor;
	auto diff = focusLength - oldLength;
	znear += diff;
	zfar += diff;
	recalculateView();
	recalculateProjection();
	repaint();
}

void GLView::mousePressEvent(QMouseEvent* e)
{
	if (e->buttons() == Qt::MiddleButton)
	{
		tracking = true;
	}
	else if (e->buttons() == Qt::RightButton)
	{
		panningTilting = true;
	}
	dragStart = e->pos();
}

void GLView::mouseMoveEvent(QMouseEvent* e)
{
	if (tracking)
	{
		glm::vec3 dir(sin(tilt) * cos(pan), sin(pan), cos(tilt) * cos(pan));
		glm::vec3 right = glm::cross(dir, glm::vec3(0, 1, 0));
		glm::vec3 up = glm::cross(right, dir);
		right = glm::normalize(right);
		up = glm::normalize(up);
		float scal = focusLength * 2 * tan(fovy * 3.1415926f / 360) / (float)height();
		focus = focus
			+ right * ((e->x() - dragStart.x()) * scal)
			+ up * ((e->y() - dragStart.y()) * scal);
		recalculateView();
	}
	else if (panningTilting)
	{
		pan += (e->y() - dragStart.y()) / (float)height() * 10;
		pan = std::min(1.5, std::max(-1.5, pan));
		tilt -= (e->x() - dragStart.x()) / (float)width() * 10;
		recalculateView();
	}
	dragStart = e->pos();
}

void GLView::mouseReleaseEvent(QMouseEvent* e)
{
	tracking = false;
	panningTilting = false;
}

void GLView::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);

	recalculateProjection();
}

void GLView::recalculateProjection()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float n = std::max(0.01f, znear);
	float f = std::max(znear + 0.01f, zfar);
	gluPerspective(fovy, (double)width() / height(), n, f);
	glMatrixMode(GL_MODELVIEW);
}

void GLView::align_to_bounding_box(glm::vec3 min, glm::vec3 max)
{
	focus = 0.5f * (min + max);
	fovy = 45;
	glm::vec3 halfDiff = 0.5f * (max - min);
	double radius = sqrt(halfDiff.x * halfDiff.x + halfDiff.y * halfDiff.y + halfDiff.z * halfDiff.z);
	float fov = fovy * std::min(1.0f, (float)width() / height());
	float d = radius / sin(fov / 2.0f * 3.1415926f / 180);
	focusLength = d;
	znear = d - radius;
	zfar = d + radius;
	recalculateProjection();
	recalculateView();
}