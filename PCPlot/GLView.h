#pragma once

#include <QOpenGLWidget>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions_4_3_Core>
#include <glm/glm.hpp>
#include <memory>

struct AxisInfo
{	
	int32_t offset;
	float min, max;

	AxisInfo(int offsetInStruct)
		: offset(offsetInStruct)
	{ }
};


class GLView : public QOpenGLWidget, public QOpenGLFunctions_4_3_Core
{
	Q_OBJECT
public:
	GLView(QWidget* parent, float eyeOffset = 0.0f, GLView* masterCam = nullptr);

	~GLView();

	void addAxis(const QString& name, int offset);
	void setData(const float* data, int samples, int strideInFloats);
	void prepare();

protected:
	void paintGL();
	void initializeGL();
	void resizeGL(int width, int height);
	
	void recalculateProjection();
	void recalculateView();

	void handleLoggedMessage(const QOpenGLDebugMessage&);

protected:
	
	std::vector<AxisInfo> axes;
	std::vector<QString> axisNames;
	
	glm::mat4 view, proj;

	QOpenGLDebugLogger glLogger;

	QOpenGLVertexArrayObject emptyVao;

	QOpenGLVertexArrayObject linesVao;
	QOpenGLBuffer axesBuffer, samplesBuffer;
	bool axesBufferDirty, samplesBufferDirty;

	std::unique_ptr<QOpenGLFramebufferObject> densityFbo;

	std::unique_ptr<QOpenGLShaderProgram> axesProgram, linesProgram, postProgram;

	bool restartDensity;
	int densityStart;

	int stride;
	int nSamples;
	const float* data;
};

