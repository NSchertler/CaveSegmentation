#include "stdafx.h"

#include "CaveDataGLView.h"
#include "CaveGLData.h"
#include "GLUtils.h"

#include <QMouseEvent>
#include <unordered_set>
#include <glm/gtc/type_ptr.hpp>

std::unique_ptr<QOpenGLShaderProgram> CaveDataGLView::clearProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveDataGLView::skyProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveDataGLView::markerProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveDataGLView::cursorProgram(nullptr);

CaveDataGLView::CaveDataGLView(ViewModel& vm, float eyeOffset, QWidget * parent, GLView* masterCam) : GLView(parent, false, eyeOffset, masterCam), vm(vm), fbo(-1), useSoftwareCursor(eyeOffset != 0)
{
	connect(&vm.caveData, &CaveGLData::meshChanged, this, &CaveDataGLView::meshChanged);
	connect(&vm.caveData, &CaveGLData::skeletonChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm, &ViewModel::markerChanged, this, &CaveDataGLView::issueRepaint);

	connect(&vm.caveData, &CaveGLData::segmentationChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm, &ViewModel::lookThroughChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm, &ViewModel::selectedPathChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm.hoveredElement, &ObservableVariable<int32_t>::changed, this, &CaveDataGLView::issueRepaint);
	connect(&vm.selectedVertex, &ObservableVariable<int32_t>::changed, this, &CaveDataGLView::issueRepaint);

	if (useSoftwareCursor)
	{
		connect(&vm.cursorPos, &ObservableVariable<glm::vec2>::changed, this, &CaveDataGLView::issueRepaint);
		setCursor(QCursor(Qt::BlankCursor));
	}

	setMouseTracking(true);

	connect(this, &GLView::CameraChanged, this, &CaveDataGLView::issueRepaint);

#ifdef NSIGHT_COMPATIBLE
	timer.setInterval(0);
	timer.start();
	timer.setSingleShot(false);
	connect(&timer, &QTimer::timeout, this, &CaveDataGLView::render);
#endif
}

CaveDataGLView::~CaveDataGLView()
{
}

void CaveDataGLView::mouseMoveEvent(QMouseEvent *e)
{
	GLView::mouseMoveEvent(e);

	if (!isPrimary)
		return;

	if (fbo >= 0)
	{
#ifndef NSIGHT_COMPATIBLE
		makeCurrent();
		auto oldHovered = vm.hoveredElement.get();
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glReadBuffer(GL_COLOR_ATTACHMENT0 + 1);
		int32_t newHovered;
		glReadPixels(e->x() + cursorOffset * width() / 2, height() - e->y(), 1, 1, GL_RED_INTEGER, GL_INT, &newHovered);
		if (newHovered != oldHovered)
			vm.hoveredElement.set(newHovered);
#else
		hoveredElement = 1;
#endif
	}

	auto localCursorPos = e->localPos();
	vm.cursorPos.set(glm::vec2(2.0f * localCursorPos.x() / width() - 1.0f, -2.0f * localCursorPos.y() / height() + 1.0f));
}

void CaveDataGLView::leaveEvent(QEvent *)
{
	if (isPrimary)
		vm.cursorPos.set(glm::vec2(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
}

void CaveDataGLView::mousePressEvent(QMouseEvent * e)
{
	if (e->buttons() == Qt::LeftButton)
	{
		if (vm.hoveredElement.get() > 0 && vm.caveData.skeleton)
		{
			if (e->modifiers() == 0)
			{
				vm.selectedVertex.set(vm.hoveredElement.get() - 1);
				vm.caveData.ResetColorLayer();
				vm.caveData.UpdateColorLayer();
				vm.selectedPath.clear();
				e->accept();
				repaint();
			}
			else if(e->modifiers() == Qt::ShiftModifier)
			{
				if (vm.selectedVertex.get() >= 0 && vm.selectedVertex.get() != vm.hoveredElement.get() - 1)
				{
					std::deque<int> additionalPath;
					std::unordered_set<int> verticesInPath;
					for (int vertex : vm.selectedPath)
					{
						verticesInPath.insert(vertex);
					}

					vm.caveData.FindPath(vm.selectedVertex.get(), vm.hoveredElement.get() - 1, additionalPath);
					for (auto vertex : additionalPath)
					{
						if (vm.selectedPath.size() > 0 && vm.selectedPath.back() == vertex)
							continue;
						vm.selectedPath.push_back(vertex);
						verticesInPath.insert(vertex);
					}

					for (int i = 0; i < vm.selectedPath.size(); ++i)
					{
						float c = 0.4f + 0.5f * i / (vm.selectedPath.size() - 1);
						vm.caveData.colorLayer.at(vm.selectedPath.at(i)) = glm::vec4(1, c, c, 1.0);
					}
					for (int i = 0; i < vm.caveData.colorLayer.size(); ++i)
					{
						if(verticesInPath.find(i) == verticesInPath.end())
							vm.caveData.colorLayer.at(i) = glm::vec4(0.5f, 0.5f, 0.5f, 1.0);
					}
					emit vm.selectedPathChanged();
					vm.caveData.UpdateColorLayer();
					e->accept();
					vm.selectedVertex.set(vm.hoveredElement.get() - 1);					
				}
			}			
		}
	}

	GLView::mousePressEvent(e);
}

void CaveDataGLView::resizeGL(int width, int height)
{
	GLView::resizeGL(width, height);
	recreatePickingResources = true;
}

void CaveDataGLView::keyPressEvent(QKeyEvent *e)
{
	if (e->key() == Qt::Key::Key_Space)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0);
		QImage img = this->grabFramebuffer();		
		img.save("screen.png");
	}
}

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

void CaveDataGLView::meshChanged()
{
	auto& min = vm.caveData.getMin();
	auto& max = vm.caveData.getMax();
	align_to_bounding_box(glm::vec3(min.x(), min.y(), min.z()), glm::vec3(max.x(), max.y(), max.z()));
	if (eyeOffset != 0)
	{		
		eyeOffset = (vm.caveData.getMax() - vm.caveData.getMin()).norm() * 0.001f * sgn(eyeOffset);
		recalculateView();
		recalculateProjection();
	}
	repaint();
}

void CaveDataGLView::skeletonChanged()
{
	repaint();
}

void CaveDataGLView::markerChanged()
{
	repaint();
}

void CaveDataGLView::segmentationChanged()
{
	repaint();
}

#ifdef NSIGHT_COMPATIBLE
void CaveDataGLView::render()
{
	repaint();
}
#endif

void CaveDataGLView::initializeGL()
{	
	vm.caveData.initGL(this, isPrimary);

	initializeOpenGLFunctions();

	if (clearProgram == nullptr)
	{
		clearProgram = MakeProgram("glsl/clear.vert", "glsl/clear.frag");
		markerProgram = MakeProgram("glsl/marker.vert", "glsl/marker.frag");
		cursorProgram = MakeProgram("glsl/cursor.vert", "glsl/cursor.frag");
	}	

	//Generate picking render targets
	glGenTextures(1, &pickingTexture);

	recreatePickingResources = true;

	cursorTexture = new QOpenGLTexture(QImage(":/icon/cursor.png").mirrored());

	float minmax[2];
	glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, minmax);
	std::cout << "min: " << minmax[0] << ", max: " << minmax[1] << std::endl;
	glGetFloatv(GL_SMOOTH_LINE_WIDTH_RANGE, minmax);
	std::cout << "min: " << minmax[0] << ", max: " << minmax[1] << std::endl;

	glLineWidth(5.0f);
	glEnable(GL_LINE_SMOOTH);

	GLView::initializeGL();
}

void CaveDataGLView::paintGL()
{
	if (recreatePickingResources)
	{
		glGetIntegerv(GL_FRAMEBUFFER_BINDING, &fbo);

		glBindTexture(GL_TEXTURE_2D, pickingTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, width(), height(), 0, GL_RED_INTEGER, GL_INT, nullptr);		
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + 1, GL_TEXTURE_2D, pickingTexture, 0);
		glBindTexture(GL_TEXTURE_2D, 0);

		recreatePickingResources = false;
	}
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + 1, GL_TEXTURE_2D, pickingTexture, 0);	

	GLenum bufs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT0 + 1 };

	glDrawBuffers(2, bufs);
	glClear(GL_DEPTH_BUFFER_BIT);
	glDepthMask(GL_FALSE);
	emptyVAO.bind();
	clearProgram->bind();
	glDrawArrays(GL_TRIANGLES, 0, 3);
	clearProgram->release();
	glDrawBuffers(1, bufs);
	glDepthMask(GL_TRUE);

	renderSky();

	if (vm.getLookThrough())
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glCullFace(GL_FRONT);
		vm.caveData.drawCave(this);
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		glCullFace(GL_BACK);
		vm.caveData.drawCave(this);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	else
	{
		glCullFace(GL_BACK);
		vm.caveData.drawCave(this);
	}
	
	vm.caveData.drawSkeleton(this);
	glDrawBuffers(2, bufs);
	vm.caveData.drawSkeletonPoints(this);
	glDrawBuffers(1, bufs);

	if(vm.hoveredElement.get() > 0)
		vm.caveData.drawCorrespondence(this, vm.hoveredElement.get() - 1);

	if (vm.selectedVertex.get() >= 0)
		vm.caveData.drawSelectedSkeletonVertex(this, vm.selectedVertex.get());

	if (!std::isnan(vm.getMarker().x))
	{
		auto MVP = glm::transpose(GetProjectionMatrix() * GetViewMatrix());
		auto m = QMatrix4x4(glm::value_ptr(MVP));

		markerProgram->bind();
		emptyVAO.bind();
		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		markerProgram->setUniformValue("mvp", m);
		markerProgram->setUniformValue("viewportDiagonal", sqrtf(viewport[2] * viewport[2] + viewport[3] * viewport[3]));
		markerProgram->setUniformValue("sizeMultiplier", 0.7f);
		markerProgram->setUniformValue("position", vm.getMarker().x, vm.getMarker().y, vm.getMarker().z, 1);
		markerProgram->setUniformValue("color", 0.8, 0.7, 0.5, 1.0);

		glDrawArrays(GL_POINTS, 0, 1);

		markerProgram->release();
		emptyVAO.release();
	}

	if (useSoftwareCursor && !std::isnan(vm.cursorPos.get().x))
	{
		glEnable(GL_DEPTH_CLAMP);
		emptyVAO.bind();
		cursorProgram->bind();
		cursorTexture->bind();
		cursorProgram->setUniformValue("posSize", vm.cursorPos.get().x + cursorOffset, vm.cursorPos.get().y, 2.0f / virtualAspectMultiplier * 32 / width(), 2.0f * 32 / height());
		cursorProgram->setUniformValue("depth", cursorDepth);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		cursorProgram->release();
		glDisable(GL_DEPTH_CLAMP);
	}	
}

