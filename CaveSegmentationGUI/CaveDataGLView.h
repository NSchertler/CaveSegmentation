#pragma once
#include "GLView.h"
#include <qglfunctions.h>
#include "CaveGLData.h"
#include "ViewModel.h"
#include <QTimer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLTexture>
#include <QKeyEvent>

//Main renderer
class CaveDataGLView : public GLView
{
public:
	CaveDataGLView(ViewModel& vm, float eyeOffset = 0.0f, QWidget * parent = Q_NULLPTR, GLView* masterCam = nullptr);
	~CaveDataGLView();

protected:
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void leaveEvent(QEvent *);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void resizeGL(int width, int height);
	virtual void keyPressEvent(QKeyEvent*);

protected slots:
	void meshChanged();	
	void skeletonChanged();
	void markerChanged();
	void segmentationChanged();
#ifdef NSIGHT_COMPATIBLE
	void render();
#endif

private:
	QOpenGLFunctions_3_3_Core gl;

	virtual void paintGL();
	virtual void initializeGL();

	static std::unique_ptr<QOpenGLShaderProgram> clearProgram;
	static std::unique_ptr<QOpenGLShaderProgram> markerProgram;
	static std::unique_ptr<QOpenGLShaderProgram> cursorProgram;

	//index texture for skeleton nodes
	GLuint pickingTexture;
	GLint fbo;

	bool recreatePickingResources;
	bool useSoftwareCursor;

	QOpenGLTexture* cursorTexture;

	ViewModel& vm;
#ifdef NSIGHT_COMPATIBLE
	QTimer timer;
#endif
};
