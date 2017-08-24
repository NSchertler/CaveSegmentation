#include "GLView.h"


#include "GLView.h"
#include <gl\GLU.h>
#include <QWheelEvent>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

#include <QMessageBox>
#include <QPainter>
#include <QTextDocument>

#include <chrono>

std::unique_ptr<QOpenGLShaderProgram> MakeProgram(QString vsPath, QString fsPath, QString gsPath = QString())
{
	auto program = std::make_unique<QOpenGLShaderProgram>();

	bool success;
	success = program->addShaderFromSourceFile(QOpenGLShader::Vertex, vsPath);
	if (!success)
	{
		QMessageBox::critical(nullptr, vsPath + " Compile Error", program->log());
		return nullptr;
	}

	if (!gsPath.isEmpty())
	{
		success = program->addShaderFromSourceFile(QOpenGLShader::Geometry, gsPath);
		if (!success)
		{
			QMessageBox::critical(nullptr, gsPath + " Compile Error", program->log());
			return nullptr;
		}
	}

	success = program->addShaderFromSourceFile(QOpenGLShader::Fragment, fsPath);
	if (!success)
	{
		QMessageBox::critical(nullptr, fsPath + " Compile Error", program->log());
		return nullptr;
	}

	success = program->link();
	if (!success)
	{
		QMessageBox::critical(nullptr, "Shader Linking Error", "VS: " + vsPath + "; GS: " + gsPath + "; FS: " + fsPath + "\n" + program->log());
		return nullptr;
	}

	return std::move(program);
}

GLView::GLView(QWidget* parent, float eyeOffset, GLView* masterCam)
	: QOpenGLWidget(parent), axesBuffer(QOpenGLBuffer::VertexBuffer), samplesBuffer(QOpenGLBuffer::VertexBuffer),
	axesBufferDirty(true), samplesBufferDirty(true), restartDensity(true)
{
	QSurfaceFormat fmt;
	fmt.setSamples(4);
#if _DEBUG
	fmt.setOption(QSurfaceFormat::DebugContext);
#endif
	setFormat(fmt);	
}

GLView::~GLView()
{
}

void GLView::addAxis(const QString & name, int offset)
{
	axes.emplace_back(offset);
	axisNames.push_back(name);
}

void GLView::setData(const float * data, int samples, int strideInFloats)
{
	this->data = data;

	stride = strideInFloats;
	nSamples = samples;

	samplesBufferDirty = true;
}

void GLView::prepare()
{
#pragma omp parallel for
	for (int iAxis = 0; iAxis < axes.size(); ++iAxis)
	{
		auto& axis = axes.at(iAxis);
		axis.min = std::numeric_limits<float>::infinity();
		axis.max = -std::numeric_limits<float>::infinity();
		for (int i = 0; i < nSamples; ++i)
		{
			float s = data[i * stride + axis.offset];
			if (s < axis.min)
				axis.min = s;
			if (s > axis.max)
				axis.max = s;
		}
		if (axis.min == axis.max)
		{
			float offset = std::max(0.01f, 0.1f * axis.min);
			axis.min -= offset;
			axis.max += offset;
		}
	}	

	axesBufferDirty = true;	
}


void GLView::paintGL()
{
	if (axesBufferDirty)
	{
		axesBuffer.bind();
		axesBuffer.allocate(axes.data(), sizeof(AxisInfo) * axes.size());
		axesBufferDirty = false;
		restartDensity = true;
	}

	if (samplesBufferDirty)
	{
		samplesBuffer.bind();
		samplesBuffer.allocate(data, sizeof(float) * nSamples * stride);
		samplesBufferDirty = false;
		restartDensity = true;
	}

	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	QFont fontBig("Cambria", 24);
	QFontMetrics fmBig(fontBig);
	QFont fontSmall("Cambria", 18);
	QFontMetrics fmSmall(fontSmall);
	QPainter painter(this);

	painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
	QColor fontColor = QColor(0, 0, 0);
	painter.setPen(QPen(fontColor, 20));
	painter.setFont(fontSmall);	

	QTextDocument td;
	td.setDefaultFont(fontBig);

	glm::mat4 viewProj = proj * view;
	glm::mat4 viewProjInv = glm::inverse(viewProj);

	for (int i = 0; i < axes.size(); ++i)
	{
		auto& axis = axes.at(i);

		glm::vec4 lower = viewProj * glm::vec4(i, 0, 0, 1);
		glm::vec4 upper = viewProj * glm::vec4(i, 1, 0, 1);

		lower.x = (lower.x + 1) * width() / 2;
		upper.x = (upper.x + 1) * width() / 2;
		lower.y = (lower.y - 1) * -height() / 2;
		upper.y = (upper.y - 1) * -height() / 2;

		
		td.setHtml(axisNames.at(i));		
		auto size = td.size();
		painter.translate(upper.x - size.width() / 2, upper.y - size.height() - fmSmall.height() - 5);
		td.drawContents(&painter);
		painter.resetTransform();

		QString txt = QString::number(axis.max);
		painter.drawText(upper.x - fmSmall.width(txt) / 2, upper.y - 10, txt);
		txt = QString::number(axis.min);
		painter.drawText(lower.x - fmSmall.width(txt) / 2, lower.y + 5 + fmSmall.height(), txt);
	}
	
	painter.end();
	
	auto MVP = glm::transpose(proj * view);
	auto mvp = QMatrix4x4(glm::value_ptr(MVP));

	axesProgram->bind();	
	axesProgram->setUniformValue("mvp", mvp);
	emptyVao.bind();
	glDrawArrays(GL_LINES, 0, 2 * axes.size());
	emptyVao.release();

	if (restartDensity)
	{
		restartDensity = false;
		densityStart = 0;

		densityFbo->bind();
		glClearColor(0, 0, 0, 0);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	if (densityStart < nSamples)
	{
		densityFbo->bind();
		glEnable(GL_BLEND);
		glBlendEquation(GL_MAX);
		glBlendFunc(GL_ONE, GL_ONE);
		linesProgram->bind();
		linesProgram->setUniformValue("mvp", mvp);
		linesProgram->setUniformValue("stride", stride);
		linesProgram->setUniformValue("qualityOffset", 0);		
		linesProgram->setUniformValue("instanceOffset", densityStart);

		GLuint64 startTime, stopTime;
		unsigned int queryID[2];

		// generate two queries
		glGenQueries(2, queryID);
		glQueryCounter(queryID[0], GL_TIMESTAMP);

		emptyVao.bind();		
		do
		{			
			int drawAmount = std::min(100000, nSamples - densityStart);
			glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 2 * axes.size(), drawAmount);
			densityStart += drawAmount;

			glQueryCounter(queryID[1], GL_TIMESTAMP);
			GLint stopTimerAvailable = 0;
			while (!stopTimerAvailable)
			{
				glGetQueryObjectiv(queryID[1],
					GL_QUERY_RESULT_AVAILABLE,
					&stopTimerAvailable);
			}
			glGetQueryObjectui64v(queryID[0], GL_QUERY_RESULT, &startTime);
			glGetQueryObjectui64v(queryID[1], GL_QUERY_RESULT, &stopTime);
		} while (densityStart < nSamples && stopTime - startTime < 200 * 1000000);
		emptyVao.release();
		densityFbo->release();
	}

	glBlendEquation(GL_FUNC_ADD);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBindTexture(GL_TEXTURE_2D, densityFbo->texture());
	postProgram->bind();
	emptyVao.bind();
	glDrawArrays(GL_TRIANGLES, 0, 3);
	emptyVao.release();
	glBindTexture(GL_TEXTURE_2D, 0);
	
	if (densityStart < nSamples)
		update();
}

void GLView::initializeGL()
{
	initializeOpenGLFunctions();

	//glEnable(GL_CULL_FACE);	
	glDisable(GL_DEPTH);

	axesProgram = MakeProgram(":/glsl/axes.vert", ":/glsl/axes.frag");
	emptyVao.create();
	emptyVao.release();

	linesProgram = MakeProgram(":/glsl/lines.vert", ":/glsl/lines.frag");
	linesVao.create();
	axesBuffer.create();	
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, axesBuffer.bufferId());
	
	samplesBuffer.create();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, samplesBuffer.bufferId());
	linesVao.release();

	postProgram = MakeProgram(":/glsl/post.vert", ":/glsl/post.frag");

	recalculateView();	

#ifdef _DEBUG
	bool loggerInitialized = glLogger.initialize();
	bool hasDebugCapability = context()->hasExtension(QByteArrayLiteral("GL_KHR_debug"));

	if (hasDebugCapability && loggerInitialized)
	{
		connect(&glLogger, &QOpenGLDebugLogger::messageLogged, this, &GLView::handleLoggedMessage);
		glLogger.disableMessages(QOpenGLDebugMessage::AnySource, QOpenGLDebugMessage::AnyType, QOpenGLDebugMessage::NotificationSeverity);
		glLogger.startLogging(QOpenGLDebugLogger::SynchronousLogging);
	}
#endif
}

void GLView::recalculateView()
{
	view = glm::mat4(1.0f);
}

void GLView::handleLoggedMessage(const QOpenGLDebugMessage & message)
{
	qDebug() << message;
}

void GLView::resizeGL(int width, int height)
{
	QOpenGLWidget::resizeGL(width, height);
	glViewport(0, 0, width, height);
	
	QOpenGLFramebufferObjectFormat fmt;
	fmt.setMipmap(false);
	fmt.setInternalTextureFormat(GL_R32F);
	densityFbo = std::make_unique<QOpenGLFramebufferObject>(width, height, fmt);

	recalculateProjection();

	restartDensity = true;
}

void GLView::recalculateProjection()
{
	proj = glm::ortho<float>(-1, axes.size(), -1, 2);
}