#include "stdafx.h"

#include "CaveDataGLView.h"
#include "CaveGLData.hpp"
#include "GLUtils.h"

#include <QMouseEvent>
#include <unordered_set>
#include <glm/gtc/type_ptr.hpp>

CaveDataGLView::CaveDataGLView(ViewModel& vm, QWidget * parent) : GLView(parent), vm(vm), fbo(-1), hoveredElement(0), selectedVertex(-1)
{
	connect(&vm.caveData, &CaveGLData::meshChanged, this, &CaveDataGLView::meshChanged);
	connect(&vm.caveData, &CaveGLData::skeletonChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm, &ViewModel::markerChanged, this, &CaveDataGLView::issueRepaint);

	connect(&vm.caveData, &CaveGLData::segmentationChanged, this, &CaveDataGLView::issueRepaint);
	connect(&vm, &ViewModel::lookThroughChanged, this, &CaveDataGLView::issueRepaint);

	setMouseTracking(true);

#ifdef NSIGHT_COMPATIBLE
	timer.setInterval(0);
	timer.start();
	timer.setSingleShot(false);
	connect(&timer, &QTimer::timeout, this, &CaveDataGLView::render);
#endif
}

CaveDataGLView::~CaveDataGLView() {
	
}

void CaveDataGLView::mouseMoveEvent(QMouseEvent *e)
{
	GLView::mouseMoveEvent(e);

	if (fbo >= 0)
	{
		makeCurrent();
		auto oldHovered = hoveredElement;
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glReadBuffer(GL_COLOR_ATTACHMENT0 + 1);
		glReadPixels(e->x(), height() - e->y(), 1, 1, GL_RED_INTEGER, GL_INT, &hoveredElement);
		if (hoveredElement != oldHovered)
			repaint();
	}
}

void CaveDataGLView::mousePressEvent(QMouseEvent * e)
{
	if (e->buttons() == Qt::LeftButton)
	{
		if (hoveredElement > 0)
		{
			if (e->modifiers() == 0)
			{
				selectedVertex = hoveredElement - 1;
				vm.caveData.ResetColorLayer();
				vm.caveData.UpdateColorLayer();
				vm.selectedPath.clear();
				e->accept();
				repaint();
			}
			else if(e->modifiers() == Qt::ShiftModifier)
			{
				if (selectedVertex >= 0 && selectedVertex != hoveredElement - 1)
				{
					std::deque<int> additionalPath;
					std::unordered_set<int> verticesInPath;
					for (int vertex : vm.selectedPath)
					{
						verticesInPath.insert(vertex);
					}

					vm.caveData.FindPath(selectedVertex, hoveredElement - 1, additionalPath);
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
					selectedVertex = hoveredElement - 1;
					repaint();

					//Debug output
					for (int i = 0; i < vm.selectedPath.size() - 1; ++i)
					{
						int v1 = vm.selectedPath.at(i);
						int v2 = vm.selectedPath.at(i + 1);
						auto edgeId = vm.caveData.vertexPairToEdge.at(std::pair<int, int>(v1, v2));
						auto edge = vm.caveData.skeleton->edges.at(edgeId);
						std::cout << "Edge " << edgeId << ": " << edge.first << " -> " << edge.second << " (" << vm.caveData.caveSizeDerivativesPerEdge.at(edgeId) << ")" << std::endl;
					}
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

void CaveDataGLView::meshChanged()
{
	align_to_bounding_box(vm.caveData.getMin(), vm.caveData.getMax());
	repaint();
}

void CaveDataGLView::issueRepaint()
{
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
	GLView::initializeGL();
	vm.caveData.initGL(this);

	initializeOpenGLFunctions();

	clearProgram = MakeProgram("clear.vert", "clear.frag");
	markerProgram = MakeProgram("marker.vert", "marker.frag");
	clearVAO.create();

	//Generate picking render targets
	glGenTextures(1, &pickingTexture);

	recreatePickingResources = true;
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
	clearVAO.bind();
	clearProgram->bind();
	glDrawArrays(GL_TRIANGLES, 0, 3);
	clearProgram->release();
	glDepthMask(GL_TRUE);
	glDrawBuffers(1, bufs);

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

	if(hoveredElement > 0)
		vm.caveData.drawCorrespondence(this, hoveredElement - 1);

	if (selectedVertex >= 0)
		vm.caveData.drawSelectedSkeletonVertex(this, selectedVertex);

	if (!std::isnan(vm.getMarker().x))
	{
		auto MVP = glm::transpose(GetProjectionMatrix() * GetViewMatrix());
		auto m = QMatrix4x4(glm::value_ptr(MVP));

		markerProgram->bind();
		clearVAO.bind();
		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		markerProgram->setUniformValue("mvp", m);
		markerProgram->setUniformValue("viewportDiagonal", sqrtf(viewport[2] * viewport[2] + viewport[3] * viewport[3]));
		markerProgram->setUniformValue("sizeMultiplier", 0.7f);
		markerProgram->setUniformValue("position", vm.getMarker().x, vm.getMarker().y, vm.getMarker().z, 1);
		markerProgram->setUniformValue("color", 0.8, 0.7, 0.5, 1.0);

		glDrawArrays(GL_POINTS, 0, 1);

		markerProgram->release();
		clearVAO.release();
	}
}

