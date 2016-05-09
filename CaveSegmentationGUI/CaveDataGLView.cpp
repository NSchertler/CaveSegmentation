#include "stdafx.h"

#include "CaveDataGLView.h"
#include "CaveGLData.hpp"

CaveDataGLView::CaveDataGLView(CaveGLData& data, QWidget * parent) : GLView(parent), data(data)
{
	connect(&data, &CaveGLData::meshChanged, this, &CaveDataGLView::meshChanged);

#ifdef NSIGHT_COMPATIBLE
	timer.setInterval(0);
	timer.start();
	timer.setSingleShot(false);
	connect(&timer, &QTimer::timeout, this, &CaveDataGLView::render);
#endif
}

CaveDataGLView::~CaveDataGLView() {
	
}

void CaveDataGLView::meshChanged()
{
	align_to_bounding_box(data.getMin(), data.getMax());
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
	data.initGL(context());

	initializeGLFunctions();
}

void CaveDataGLView::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	/*glBegin(GL_LINES);

	double size = glm::length(data.getMax() - data.getMin()) * 0.2;
	glm::vec3 center(0, 0, 0);
	if (isinf(size))
	{
		size = 1;
	}
	else
	{
		center = (data.getMax() + data.getMin()) * 0.5f;
	}


	glColor3f(1, 0, 0);
	glVertex3f(center.x, center.y, center.z);
	glVertex3f(center.x + size, center.y, center.z);

	glColor3f(0, 1, 0);
	glVertex3f(center.x, center.y, center.z); 
	glVertex3f(center.x, center.y + size, center.z);

	glColor3f(0, 0, 1);
	glVertex3f(center.x, center.y, center.z);
	glVertex3f(center.x, center.y, center.z + size);

	glEnd();*/

	data.drawCave();
}

