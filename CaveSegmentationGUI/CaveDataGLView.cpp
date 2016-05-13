#include "stdafx.h"

#include "CaveDataGLView.h"
#include "CaveGLData.hpp"
#include "GLUtils.h"

#include <QMouseEvent>
#include <unordered_set>

CaveDataGLView::CaveDataGLView(CaveGLData& data, QWidget * parent) : GLView(parent), data(data), fbo(-1), hoveredElement(0), selectedVertex(-1)
{
	connect(&data, &CaveGLData::meshChanged, this, &CaveDataGLView::meshChanged);
	connect(&data, &CaveGLData::skeletonChanged, this, &CaveDataGLView::skeletonChanged);

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
				data.ResetColorLayer();
				data.UpdateColorLayer();
				path.clear();
				e->accept();
				repaint();
			}
			else if(e->modifiers() == Qt::ShiftModifier)
			{
				if (selectedVertex >= 0 && selectedVertex != hoveredElement - 1)
				{
					std::deque<int> additionalPath;
					std::unordered_set<int> verticesInPath;
					for (int vertex : path)
					{
						verticesInPath.insert(vertex);
					}

					data.FindPath(selectedVertex, hoveredElement - 1, additionalPath);
					for (auto vertex : additionalPath)
					{
						if (path.size() > 0 && path.back() == vertex)
							continue;
						path.push_back(vertex);
						verticesInPath.insert(vertex);
					}

					for (int i = 0; i < path.size(); ++i)
					{
						float c = 0.4f + 0.5f * i / (path.size() - 1);
						data.colorLayer.at(path.at(i)) = glm::vec4(1, c, c, 1.0);						
					}
					for (int i = 0; i < data.colorLayer.size(); ++i)
					{
						if(verticesInPath.find(i) == verticesInPath.end())
							data.colorLayer.at(i) = glm::vec4(0.5f, 0.5f, 0.5f, 1.0);
					}
					data.UpdateColorLayer();
					e->accept();
					selectedVertex = hoveredElement - 1;
					repaint();
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
	align_to_bounding_box(data.getMin(), data.getMax());
	repaint();
}

void CaveDataGLView::skeletonChanged()
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
	data.initGL(this);

	initializeOpenGLFunctions();

	clearProgram = MakeProgram("clear.vert", "clear.frag");
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

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCullFace(GL_FRONT);
	data.drawCave(this);
	glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
	glCullFace(GL_BACK);
	data.drawCave(this);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	glDrawBuffers(2, bufs);
	data.drawSkeleton(this);
	glDrawBuffers(1, bufs);

	if(hoveredElement > 0)
		data.drawCorrespondence(this, hoveredElement - 1);

	if (selectedVertex >= 0)
		data.drawSelectedSkeletonVertex(this, selectedVertex);

}

