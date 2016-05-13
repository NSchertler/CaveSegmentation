#pragma once
#include "GLView.h"
#include <qglfunctions.h>
#include "CaveGLData.hpp"
#include <QTimer>
#include <QOpenGLFunctions_4_3_Core>

class CaveDataGLView : public GLView, QOpenGLFunctions_4_3_Core {
	

public:
	CaveDataGLView(CaveGLData& data, QWidget * parent = Q_NULLPTR);
	~CaveDataGLView();

protected:
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void resizeGL(int width, int height);

protected slots:
	void meshChanged();
	void skeletonChanged();
#ifdef NSIGHT_COMPATIBLE
	void render();
#endif

private:
	virtual void paintGL();
	virtual void initializeGL();

	std::unique_ptr<QOpenGLShaderProgram> clearProgram;
	QOpenGLVertexArrayObject clearVAO;
	GLuint pickingTexture;
	GLint fbo;
	int32_t hoveredElement;

	int selectedVertex;
	std::vector<int> path;

	bool recreatePickingResources;

	CaveGLData& data;
#ifdef NSIGHT_COMPATIBLE
	QTimer timer;
#endif
};
