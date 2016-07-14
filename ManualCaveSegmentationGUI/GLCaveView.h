#pragma once

#include <GLView.h>
#include <QOpenGLFunctions_3_3_Core>
#include "Mesh.h"
#include "SharedData.h"

enum SegmentationType
{
	Chamber = 1,
	Passage = 2,
};

class GLCaveView : public GLView, public QOpenGLFunctions_3_3_Core
{
public:
	GLCaveView(QWidget* parent, std::shared_ptr<SharedData>);
	~GLCaveView(void);

	void setBrushSize(int v);
	void setBrushType(SegmentationType);

protected:
	virtual void paintGL();
	virtual void initializeGL();

	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
	void mouseMoveEvent(QMouseEvent*);

	void updateCursor();
	QColor chamberColor, passageColor;

	int brushSize;
	SegmentationType brushType;

	std::shared_ptr<SharedData> data;
	bool painting;
	bool measuring;
	glm::vec3 measureStart, measureEnd;

protected slots:
	void data_MeshChanged();
};

