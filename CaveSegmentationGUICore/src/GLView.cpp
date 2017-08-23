#include "GLView.h"


#include "GLView.h"
#include "GLUtils.h"
#include <gl\GLU.h>
#include <QWheelEvent>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

GLView::GLView(QWidget* parent, bool automaticDepthBufferRetrieval, float eyeOffset, GLView* masterCam)
	: QOpenGLWidget(parent), eyeOffset(eyeOffset), zeroParallaxInterpol(0.33f), cursorDepthInterpol(0.4f), 
	_depthBuffer(nullptr), depthBufferSize(0), _enableAutomaticDepthBufferRetrieval(automaticDepthBufferRetrieval),
	cameraControlModifier(Qt::NoModifier), cameraZoomModifier(Qt::AltModifier), frontViewCutoff(0), cameraHold(nullptr)
{
	Q_INIT_RESOURCE(guicore);

	fovy = 45;
	znear = 1;
	zfar = 10;
	focusLength = 5;
	focus = glm::vec3(0);

	pan = 0.3;
	tilt = 0;

	panningTilting = false;
	tracking = false;
	zooming = false;

	QSurfaceFormat fmt;
	fmt.setVersion(4, 3);
#ifdef _DEBUG
	fmt.setProfile(QSurfaceFormat::CoreProfile);
	fmt.setOptions(QSurfaceFormat::DebugContext);
#endif
	//fmt.setSamples(16);
	setFormat(fmt);

	isPrimary = masterCam == nullptr;
	if (masterCam != nullptr)
	{
		connect(masterCam, &GLView::camParamsChanged, this, &GLView::setCameraParameters);
		connect(masterCam, &GLView::zRangeChanged, this, &GLView::setZRange);

		setCameraParameters(masterCam->pan, masterCam->tilt, masterCam->focus, masterCam->focusLength);
		setZRange(masterCam->znear, masterCam->zfar, masterCam->zeroParallaxInterpol, masterCam->cursorDepthInterpol);
	}

	virtualAspectMultiplier = 1.0f;

	stillViewTimer.setInterval(500);
	stillViewTimer.setSingleShot(true);
	connect(&stillViewTimer, &QTimer::timeout, this, &GLView::stillViewTimer_timeout);
}

GLView::~GLView()
{
	if (_depthBuffer)
		delete[] _depthBuffer;
}

void GLView::setCameraControlModifier(Qt::KeyboardModifier m)
{
	cameraControlModifier = m;
}

void GLView::readDepthBuffer()
{
	makeCurrent();
	//Read the depth buffer
	int targetSize = width() * height();
	if (targetSize > depthBufferSize)
	{
		if (_depthBuffer)
			delete[] _depthBuffer;
		depthBufferSize = targetSize;
		_depthBuffer = new float[targetSize];
	}
	glReadPixels(0, 0, width(), height(), GL_DEPTH_COMPONENT, GL_FLOAT, _depthBuffer);
}

const float * GLView::depthBuffer() const
{
	return _depthBuffer;
}

bool GLView::isAutomaticDepthBufferRetrievalEnabled() const
{
	return _enableAutomaticDepthBufferRetrieval;
}

void GLView::enableAutomaticDepthBufferRetrieval()
{
	_enableAutomaticDepthBufferRetrieval = true;
	readDepthBuffer();
}

void GLView::disableAutomaticDepthBufferRetrieval()
{
	_enableAutomaticDepthBufferRetrieval = false;
}

float GLView::getZNear() const
{
	return znear;
}

float GLView::getZFar() const
{
	return zfar;
}

glm::vec3 GLView::getEye() const
{
	glm::vec3 dir(cos(tilt) * cos(pan), sin(tilt) * cos(pan), sin(pan));
	return focus + focusLength * dir;
}

void GLView::setFrontViewCutoff(float c)
{
	frontViewCutoff = c;
	recalculateProjection();
}

void GLView::findPositionUnderMouse(int x, int y, glm::vec3& v)
{
	glm::mat4 MVP = GetProjectionMatrix() * GetViewMatrix();

	auto invMVP = glm::inverse(MVP);
	float clipX = 2.0f * x / width() - 1.0f;
	float clipY = 1.0f - 2.0f * y / height();
	float depth = 2.0f * _depthBuffer[x + width() * (height() - y)] - 1.0f;

	glm::vec4 worldSpaceCoords = invMVP * glm::vec4(clipX, clipY, depth, 1);
	worldSpaceCoords *= 1.0f / worldSpaceCoords.w;
	v.x = worldSpaceCoords.x;
	v.y = worldSpaceCoords.y;
	v.z = worldSpaceCoords.z;
}

void GLView::issueRepaint()
{
	repaint();
}

void GLView::setCameraParameters(float pan, float tilt, const glm::vec3& focus, float focusLength)
{
	this->pan = pan;
	this->tilt = tilt;
	this->focus = focus;
	this->focusLength = focusLength;
	recalculateView();
}

void GLView::setZRange(float znear, float zfar, float zeroParallaxInterpol, float cursorDepthInterpol)
{
	this->znear = znear;
	this->zfar = zfar;
	this->zeroParallaxInterpol = zeroParallaxInterpol;
	this->cursorDepthInterpol = cursorDepthInterpol;
	recalculateProjection();
}

void GLView::stillViewTimer_timeout()
{
	if(_enableAutomaticDepthBufferRetrieval)
		readDepthBuffer();

	emit CameraStill();
}

void GLView::setVirtualAspectMultiplier(float multiplier)
{
	virtualAspectMultiplier = multiplier;
	recalculateProjection();
}

const glm::mat4 & GLView::GetViewMatrix()
{
	return view;
}

glm::mat4 GLView::GetViewRotationMatrix()
{
	glm::mat4 rot = view;
	rot[3][0] = 0.0f;
	rot[3][1] = 0.0f;
	rot[3][2] = 0.0f;
	return rot;
}

const glm::mat4 & GLView::GetProjectionMatrix()
{
	return proj;
}

void GLView::MakeOpenGLContextCurrent()
{
	makeCurrent();
}

void GLView::emitCameraChanged()
{
	if (cameraHold == nullptr)
		emit CameraChanged();
	else
		cameraHold->cameraChanged = true;
}

void GLView::emitCamParamsChanged()
{
	if (cameraHold == nullptr)
		emit camParamsChanged(pan, tilt, focus, focusLength);
	else
		cameraHold->camParamsChanged = true;
}

void GLView::emitZRangeChanged()
{
	if (cameraHold == nullptr)
		emit zRangeChanged(znear, zfar, zeroParallaxInterpol, cursorDepthInterpol);
	else
		cameraHold->zRangeChanged = true;
}

void GLView::initializeGL()
{
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_CULL_FACE);

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_MULTISAMPLE);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	skyProgram = MakeProgram(":/glsl/sky.vert", ":/glsl/sky.frag");
	emptyVAO.create();

	recalculateView();

#ifdef _DEBUG
	if (isPrimary)
	{
		bool loggerInitialized = glLogger.initialize();
		bool hasDebugCapability = context()->hasExtension(QByteArrayLiteral("GL_KHR_debug"));

		if (hasDebugCapability && loggerInitialized)
		{
			connect(&glLogger, &QOpenGLDebugLogger::messageLogged, this, &GLView::handleLoggedMessage);			
			glLogger.disableMessages(QOpenGLDebugMessage::AnySource, QOpenGLDebugMessage::AnyType, QOpenGLDebugMessage::NotificationSeverity);
			glLogger.startLogging(QOpenGLDebugLogger::SynchronousLogging);
		}
	}
#endif

	emit inited(this);
}

void GLView::recalculateView()
{
	glm::vec3 eye = getEye();
	view = glm::translate(glm::mat4(1.0f), glm::vec3(-eyeOffset, 0, 0)) * glm::lookAtRH(eye, focus, glm::vec3(0, 0, 1));
	emitCameraChanged();
	emitCamParamsChanged();

	stillViewTimer.start();
}

void GLView::handleLoggedMessage(const QOpenGLDebugMessage & message)
{
	qDebug() << message;
}

void GLView::zoom(float amount)
{
	auto factor = pow(0.999, amount);
	auto oldLength = focusLength;
	focusLength *= factor;
	auto diff = focusLength - oldLength;
	znear += diff;
	zfar += diff;
	recalculateView();
	emitCameraChanged();
	recalculateProjection();
	emitZRangeChanged();
}

void GLView::wheelEvent(QWheelEvent* e)
{
	HoldCameraEvents hold(this);
	if (e->modifiers() == Qt::ControlModifier)
	{
		zeroParallaxInterpol += 0.0001 * e->delta();
		if (zeroParallaxInterpol < 0)
			zeroParallaxInterpol = 0;
		if (zeroParallaxInterpol > 1)
			zeroParallaxInterpol = 1;
	}
	else if (e->modifiers() == Qt::ShiftModifier)
	{
		cursorDepthInterpol += 0.0001 * e->delta();
		if (cursorDepthInterpol < 0)
			cursorDepthInterpol = 0;
		if (cursorDepthInterpol > 1)
			cursorDepthInterpol = 1;
	}
	else
	{
		zoom(e->delta());		
	}
	recalculateProjection();
	emitZRangeChanged();
}

void GLView::mousePressEvent(QMouseEvent* e)
{
	this->setFocus();
	if (e->modifiers() == cameraControlModifier)
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
	if (e->modifiers() == cameraZoomModifier)
	{
		if (e->buttons() == Qt::LeftButton)
			zooming = true;
		dragStart = e->pos();
	}
}

void GLView::mouseMoveEvent(QMouseEvent* e)
{
	HoldCameraEvents hold(this);
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
	else if (zooming)
	{
		zoom(-e->y() + dragStart.y());
	}
	dragStart = e->pos();
}

void GLView::mouseReleaseEvent(QMouseEvent* e)
{
	tracking = false;
	panningTilting = false;
	zooming = false;
}

void GLView::renderSky()
{
	glDepthMask(GL_FALSE);

	emptyVAO.bind();
	skyProgram->bind();
	glEnable(GL_DEPTH_CLAMP);
	auto MVP = glm::transpose(GetProjectionMatrix() * GetViewRotationMatrix());
	auto mvp = QMatrix4x4(glm::value_ptr(MVP));
	skyProgram->setUniformValue("mvp", mvp);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 6);
	skyProgram->release();
	emptyVAO.release();
	glDisable(GL_DEPTH_CLAMP);

	glDepthMask(GL_TRUE);
}

void GLView::resizeGL(int width, int height)
{
	QOpenGLWidget::resizeGL(width, height);
	glViewport(0, 0, width, height);

	recalculateProjection();
}

glm::mat4 StereoFrustumScreen(float eyeOffset, float fovy,
	float aspect,
	float zZeroParallax,
	float zNear, float zFar)
{
	float top = zNear * tan(fovy / 2);
	float bottom = -top;
	float left = bottom * aspect - eyeOffset * zNear /zZeroParallax;
	float right = top*aspect - eyeOffset * zNear / zZeroParallax;
	return glm::frustum(left, right, bottom, top, zNear, zFar);
}

void GLView::recalculateProjection()
{
	float n = std::max(0.1f, (1 - frontViewCutoff) * znear + frontViewCutoff * zfar);
	float f = std::max(n, zfar);
	//proj = glm::perspectiveFovRH<float>(fovy, width(), height(), n, f);
	float zeroParallax = (1 - zeroParallaxInterpol) * n + zeroParallaxInterpol * f;
	float aspect = width() * virtualAspectMultiplier / height();
	proj = StereoFrustumScreen(eyeOffset, fovy, aspect, zeroParallax, n, f);

	float cursorZ = (1.0f - cursorDepthInterpol) * n + cursorDepthInterpol * focusLength;
	cursorOffset = eyeOffset * (1.0f - zeroParallax / cursorZ) / (tan(fovy / 2) * zeroParallax * aspect);	
	cursorDepth = (-cursorZ * proj[2][2] + proj[3][2]) / (-cursorZ * proj[2][3] + proj[3][3]);

	stillViewTimer.start();

	emitCameraChanged();
}

void GLView::align_to_bounding_box(const glm::vec3& min, const glm::vec3& max)
{
	if (!isPrimary)
		return;
	focus = 0.5f * (min + max);
	fovy = 45;
	glm::vec3 halfDiff = 0.5f * (max - min);
	double radius = sqrt(halfDiff.x * halfDiff.x + halfDiff.y * halfDiff.y + halfDiff.z * halfDiff.z);
	float fov = fovy * std::min(1.0f, (float)width() * virtualAspectMultiplier / height());
	float d = radius / sin(fov / 2.0f * 3.1415926f / 180);
	focusLength = d;
	znear = d - radius;
	zfar = d + radius;
	recalculateProjection();
	recalculateView();
	emitZRangeChanged();
}

HoldCameraEvents::HoldCameraEvents(GLView * view)
	: cameraChanged(false), camParamsChanged(false), zRangeChanged(false)
{
	if (view->cameraHold == nullptr)
	{
		this->view = view;
		view->cameraHold = this;
	}
}

HoldCameraEvents::~HoldCameraEvents()
{
	if (view)
	{
		if (cameraChanged)
			emit view->CameraChanged();
		if (camParamsChanged)
			emit view->camParamsChanged(view->pan, view->tilt, view->focus, view->focusLength);
		if(zRangeChanged)
			emit view->zRangeChanged(view->znear, view->zfar, view->zeroParallaxInterpol, view->cursorDepthInterpol);
		view->cameraHold = nullptr;
	}
}
