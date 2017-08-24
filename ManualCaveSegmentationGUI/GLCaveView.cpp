#include "GLCaveView.h"
#include <glm/gtc/type_ptr.hpp>
#include <QMouseEvent>
#include <QPainter>

GLCaveView::GLCaveView(QWidget* parent, std::shared_ptr<SharedData> data) : GLView(parent, true), measuring(false)
{
	painting = false;

	chamberColor = QColor(175, 0, 0, 128);
	passageColor = QColor(0, 175, 0, 128);
	eraseColor = QColor(255, 255, 255, 128);

	this->data = data;

	setCameraControlModifier(Qt::ControlModifier);

	connect(data.get(), &SharedData::meshChanged, this, &GLCaveView::data_MeshChanged);
	connect(this, &GLView::CameraChanged, this, &GLCaveView::issueRepaint);
}

void GLCaveView::data_MeshChanged()
{
	align_to_bounding_box(data->getMesh()->getMin(), data->getMesh()->getMax());
	repaint();

	makeCurrent();
	data->getMesh()->createBuffers();
}

void GLCaveView::updateCursor()
{
	QPixmap m_LPixmap(2 * brushSize, 2 * brushSize);
	m_LPixmap.fill(Qt::transparent);
	QPainter painter(&m_LPixmap);
	

	painter.setPen(Qt::NoPen);
	switch (brushType)
	{
	case Chamber: painter.setBrush(chamberColor); break;
	case Passage: painter.setBrush(passageColor); break;
	case Erase: painter.setBrush(eraseColor); break;
	}	
	
	painter.drawEllipse(0, 0, 2 * brushSize, 2 * brushSize);
	auto m_Cursor = QCursor(m_LPixmap);
	setCursor(m_Cursor);
}


void GLCaveView::setBrushSize(int v)
{
	brushSize = v;	
	updateCursor();
}

void GLCaveView::setBrushType(SegmentationType v)
{
	brushType = v;
	updateCursor();
}

GLCaveView::~GLCaveView(void)
{
}

void GLCaveView::initializeGL()
{
	initializeOpenGLFunctions();

	GLView::initializeGL();
	
	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glDisable(GL_CULL_FACE);

	glClearColor(0.2f, 0.2f, 0.6f, 1.0f);

	Mesh::init_shaders(this->context());	
}

void GLCaveView::mousePressEvent(QMouseEvent* e)
{		
	if (data->getMesh() && !tracking && !panningTilting && e->modifiers() == 0)
	{
		data->getMesh()->paint(e->x(), height() - e->y(), brushSize, (int)brushType);
		painting = true;
	}
	GLView::mousePressEvent(e);
}

void GLCaveView::mouseReleaseEvent(QMouseEvent* e)
{
	GLView::mouseReleaseEvent(e);
	painting = false;
}

void GLCaveView::mouseMoveEvent(QMouseEvent* e)
{
	if (painting && data->getMesh() && !tracking && !panningTilting && e->modifiers() == 0)
		data->getMesh()->paint(e->x(), height() - e->y(), brushSize, (int)brushType);
	GLView::mouseMoveEvent(e);
}

void GLCaveView::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	
	renderSky();

	if (data->getMesh())
	{
		data->getMesh()->draw(this, measuring);

		int northX, northY, eastX, eastY;
		data->getMesh()->getAxisEnds(this, northX, northY, eastX, eastY);
	}
}