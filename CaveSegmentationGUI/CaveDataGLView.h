#pragma once
#include "GLView.h"
#include <qglfunctions.h>
#include "CaveGLData.hpp"
#include "ViewModel.h"
#include <QTimer>
#include <QOpenGLFunctions_3_3_Core>

class CaveDataGLView : public GLView, QOpenGLFunctions_3_3_Core {
	

public:
	CaveDataGLView(ViewModel& vm, QWidget * parent = Q_NULLPTR);
	~CaveDataGLView();

protected:
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void resizeGL(int width, int height);

protected slots:
	void meshChanged();
	void issueRepaint();
	void skeletonChanged();
	void markerChanged();
	void segmentationChanged();
#ifdef NSIGHT_COMPATIBLE
	void render();
#endif

private:
	virtual void paintGL();
	virtual void initializeGL();

	std::unique_ptr<QOpenGLShaderProgram> clearProgram;
	std::unique_ptr<QOpenGLShaderProgram> markerProgram;
	QOpenGLVertexArrayObject clearVAO;
	GLuint pickingTexture;
	GLint fbo;
	int32_t hoveredElement;

	int selectedVertex;	

	bool recreatePickingResources;

	ViewModel& vm;
#ifdef NSIGHT_COMPATIBLE
	QTimer timer;
#endif
};
