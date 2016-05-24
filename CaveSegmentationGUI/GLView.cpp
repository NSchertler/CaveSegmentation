#include "stdafx.h"

#include "GLView.h"


#include "GLView.h"
#include <gl\GLU.h>
#include <QWheelEvent>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

GLView::GLView(QWidget* parent) : QOpenGLWidget(parent)
{
	fovy = 45;
	znear = 1;
	zfar = 10;
	focusLength = 5;
	focus = glm::vec3(0);

	pan = M_PI / 4;
	tilt = 0;

	panningTilting = false;
	tracking = false;

	QSurfaceFormat fmt;
	fmt.setVersion(4, 3);
	fmt.setProfile(QSurfaceFormat::CoreProfile);
	fmt.setOptions(QSurfaceFormat::DebugContext);
	//fmt.setSamples(16);
	setFormat(fmt);
}

const glm::mat4 & GLView::GetViewMatrix()
{
	return view;
}

const glm::mat4 & GLView::GetProjectionMatrix()
{
	return proj;
}

void GLView::MakeOpenGLContextCurrent()
{
	makeCurrent();
}

void GLView::initializeGL()
{
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_CULL_FACE);

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_MULTISAMPLE);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	recalculateView();

#ifdef _DEBUG
	bool loggerInitialized = glLogger.initialize();
	bool hasDebugCapability = context()->hasExtension(QByteArrayLiteral("GL_KHR_debug"));

	if (hasDebugCapability && loggerInitialized)
	{
		connect(&glLogger, &QOpenGLDebugLogger::messageLogged, this, &GLView::handleLoggedMessage);
		glLogger.disableMessages(QOpenGLDebugMessage::AnySource, QOpenGLDebugMessage::AnyType, QOpenGLDebugMessage::NotificationSeverity);
		glLogger.startLogging(QOpenGLDebugLogger::SynchronousLogging);
	}
#endif

	emit inited(this);
}

void GLView::recalculateView()
{
	glm::vec3 dir(cos(tilt) * cos(pan), sin(tilt) * cos(pan), sin(pan));
	glm::vec3 eye = focus + focusLength * dir;
	view = glm::lookAtRH(eye, focus, glm::vec3(0, 0, 1));
	repaint();
}

void GLView::handleLoggedMessage(const QOpenGLDebugMessage & message)
{
	qDebug() << message;
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
	if (e->buttons() == Qt::RightButton)
	{
		tracking = true;
	}
	else if (e->buttons() == Qt::LeftButton)
	{
		panningTilting = true;
	}
	dragStart = e->pos();
}

void GLView::mouseMoveEvent(QMouseEvent* e)
{
	if (tracking)
	{
		glm::vec3 dir(cos(tilt) * cos(pan), sin(tilt) * cos(pan), sin(pan));
		glm::vec3 right = glm::cross(dir, glm::vec3(0, 0, 1));
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
	QOpenGLWidget::resizeGL(width, height);
	glViewport(0, 0, width, height);

	recalculateProjection();
}

void GLView::recalculateProjection()
{
	float n = std::max(0.01f, znear);
	float f = std::max(znear + 0.01f, zfar);
	proj = glm::perspectiveFovRH<float>(fovy, width(), height(), n, f);
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