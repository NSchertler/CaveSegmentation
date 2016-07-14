#include "GLView.h"


#include "GLView.h"
#include <gl\GLU.h>
#include <QWheelEvent>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

GLView::GLView(QWidget* parent) : QGLWidget(parent), nearClip(0), depthBuffer(nullptr), depthBufferSize(0), measuring(false)
{
	virtualScreenHeight = 5;
	znear = 1;
	zfar = 10;
	focusLength = 5;
	focus = glm::vec3(0);

	pan = -0.61547970867;
	tilt = 0.7853981634;

	panningTilting = false;
	tracking = false;

	timer.setSingleShot(true);
	timer.setInterval(250);
	connect(&timer, &QTimer::timeout, this, &GLView::timer_timeout);

	setMouseTracking(true);
}

GLView::~GLView()
{
	if (depthBuffer)
		delete[] depthBuffer;
}

void GLView::issueViewChanged()
{
	timer.start();
}

void GLView::readDepthBuffer()
{
	//Read the depth buffer
	int targetSize = width() * height();
	if (targetSize > depthBufferSize)
	{
		if (depthBuffer)
			delete[] depthBuffer;
		depthBufferSize = targetSize;
		depthBuffer = new float[targetSize];
	}
	glReadPixels(0, 0, width(), height(), GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer);
}

void GLView::timer_timeout()
{
	readDepthBuffer();

	emit viewUpdated();
}

void GLView::initializeGL()
{
	glClearColor(0, 0.1f, 0.3f, 0);

	glEnable(GL_PROGRAM_POINT_SIZE);

	glEnable(GL_DEPTH_TEST); 	

	recalculateView();

	emit inited(this);
}

void GLView::recalculateView()
{
	glm::vec3 dir(sin(tilt) * cos(pan), sin(pan), cos(tilt) * cos(pan));
	glm::vec3 eye = focus + focusLength * dir;
	glLoadIdentity();
	
	glRotatef(-pan * 180.0f / 3.1415f, 1, 0, 0);
	glRotatef(-tilt * 180.0f / 3.1415f, 0, 1, 0);
	glRotatef(-90, 1, 0, 0);
	glTranslatef(-focus.x, -focus.y, -focus.z);
	
	repaint();
}

void GLView::setNearClip(float value)
{
	nearClip = value;
	recalculateProjection();
	issueViewChanged();

	repaint();
}

void GLView::wheelEvent(QWheelEvent* e)
{
	auto factor = pow(0.999, e->delta());
	virtualScreenHeight *= factor;
	recalculateProjection();
	repaint();

	issueViewChanged();
}

void GLView::mousePressEvent(QMouseEvent* e)
{
	if (e->modifiers() == 0)
	{
		if (e->buttons() == Qt::MiddleButton)
		{
			tracking = true;
		}
		else if (e->buttons() == Qt::RightButton)
		{
			panningTilting = true;
		}
	}
	else if (e->modifiers() == Qt::ShiftModifier && e->buttons() == Qt::LeftButton)
	{		
		findPositionUnderMouse(e->x(), e->y(), measureStart);
		measureEnd = measureStart;
		measuring = true;
		repaint();
	}
	else if (e->modifiers() == Qt::ShiftModifier && e->buttons() == Qt::RightButton)
	{
		measuring = false;
		repaint();
	}
	dragStart = e->pos();	
}

void GLView::findPositionUnderMouse(int x, int y, glm::vec3& v)
{
	glm::mat4 modelView, proj, MVP;
	glGetFloatv(GL_MODELVIEW_MATRIX, glm::value_ptr(modelView));
	glGetFloatv(GL_PROJECTION_MATRIX, glm::value_ptr(proj));
	MVP = proj * modelView;

	auto invMVP = glm::inverse(MVP);
	float clipX = 2.0f * x / width() - 1.0f;
	float clipY = 1.0f - 2.0f * y / height();
	float depth = 2.0f * depthBuffer[x + width() * (height() - y)] - 1.0f;

	glm::vec4 worldSpaceCoords = invMVP * glm::vec4(clipX, clipY, depth, 1);
	worldSpaceCoords *= 1.0f / worldSpaceCoords.w;
	v.x = worldSpaceCoords.x;
	v.y = worldSpaceCoords.y;
	v.z = worldSpaceCoords.z;
}

void GLView::mouseMoveEvent(QMouseEvent* e)
{	
	if (e->buttons() != 0)
	{
		float viewM[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, viewM);
		if (tracking)
		{
			glm::vec3 right(viewM[0], viewM[4], viewM[8]);
			glm::vec3 up(viewM[1], viewM[5], viewM[9]);
			float scal = virtualScreenHeight / (float)height();
			focus = focus
				- right * ((e->x() - dragStart.x()) * scal)
				+ up * ((e->y() - dragStart.y()) * scal);

			recalculateView();
		}
		else if (panningTilting)
		{
			pan -= (e->y() - dragStart.y()) / (float)height() * 10;
			pan = std::min(1.5, std::max(-1.5, pan));
			tilt -= (e->x() - dragStart.x()) / (float)width() * 10;
			if (tilt < -3.1415926)
				tilt += 6.2831853071;
			if (tilt > 3.1415926)
				tilt -= 6.2831853071;
			recalculateView();
		}
		if (tracking || panningTilting)
		{
			//update znear, zfar
			glGetFloatv(GL_MODELVIEW_MATRIX, viewM);
			glm::vec3 viewDir(-viewM[2], -viewM[6], -viewM[10]);
			float distToObject = glm::dot(objectCenter - focus, viewDir);
			znear = distToObject - objectRadius;
			zfar = distToObject + objectRadius;
			recalculateProjection();

			issueViewChanged();
		}

		dragStart = e->pos();
	}
	else
	{
		if (measuring)
		{
			findPositionUnderMouse(e->x(), e->y(), measureEnd);
			repaint();
		}
	}
}

void GLView::mouseReleaseEvent(QMouseEvent* e)
{
	tracking = false;
	panningTilting = false;
	issueViewChanged();
}

void GLView::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);

	recalculateProjection();

	issueViewChanged();
}

void GLView::recalculateProjection()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(fovy, (double)width() / height(), n, f);
	double ratio = (double)width() / height();
	//glOrtho(-ratio * virtualScreenHeight / 2, -ratio * virtualScreenHeight / 2, virtualScreenHeight / 2, -virtualScreenHeight / 2, znear, zfar);
	znear = objectRadius * (2 * nearClip - 1);
	glOrtho(-ratio * virtualScreenHeight / 2, ratio * virtualScreenHeight / 2, -virtualScreenHeight / 2, virtualScreenHeight / 2, znear, zfar);
	glMatrixMode(GL_MODELVIEW);
}

void GLView::align_to_bounding_box(glm::vec3 min, glm::vec3 max)
{
	objectCenter = 0.5f * (min + max);
	focus = objectCenter;
	//fovy = 45;
	glm::vec3 halfDiff = 0.5f * (max - min);
	objectRadius = sqrt(halfDiff.x * halfDiff.x + halfDiff.y * halfDiff.y + halfDiff.z * halfDiff.z);
	virtualScreenHeight = 2 * objectRadius;
	focusLength = 0;	
	zfar = objectRadius;
	//float fov = fovy * std::min(1.0f, (float)width()/height());
	//float d = radius / sin(fov/2.0f * 3.1415926f / 180);
	//focusLength = d;
	//znear = d - radius;
	//zfar = d + radius;
	recalculateProjection();
	recalculateView();

	issueViewChanged();
}