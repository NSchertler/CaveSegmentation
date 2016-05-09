#pragma once
#include <QObject>
#include <qglfunctions.h>
#include <qopenglbuffer.h>
#include <QGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include "CaveData.h"
#include "IHasBoundingBox.h"

class CaveGLData : public QObject, public CaveData, QGLFunctions, public IHasBoundingBox {
	Q_OBJECT

public:
	CaveGLData(QObject * parent = Q_NULLPTR);
	~CaveGLData();

	virtual void LoadMesh(const std::string& offFile);

	void drawCave();

	void initGL(const QGLContext* ctx);
	static void init_shaders(const QGLContext* context);

signals:
	void meshChanged();

private:
	QOpenGLBuffer meshVB, meshIB;
	QOpenGLVertexArrayObject meshVAO;

	static std::unique_ptr<QGLShaderProgram> program;
};
