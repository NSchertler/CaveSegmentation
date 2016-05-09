#include "stdafx.h"

#include "CaveGLData.hpp"
#include <qmessagebox.h>
#include <glm/gtc/type_ptr.hpp>

std::unique_ptr<QGLShaderProgram> CaveGLData::program(nullptr);

CaveGLData::CaveGLData(QObject * parent)
	: QObject(parent), meshVB(QOpenGLBuffer::VertexBuffer), meshIB(QOpenGLBuffer::IndexBuffer)
{	
}

CaveGLData::~CaveGLData()
{
	meshVB.destroy();
	meshIB.destroy();
}

void CaveGLData::LoadMesh(const std::string & offFile)
{
	CaveData::LoadMesh(offFile);

	meshVAO.create();
	meshVAO.bind();

	meshVB.bind();
	meshVB.allocate(&_meshVertices[0], sizeof(Eigen::Vector3f) * _meshVertices.size());
	program->enableAttributeArray("in_position");
	program->setAttributeBuffer("in_position", GL_FLOAT, 0, 3);

	meshIB.bind();
	int* indices = reinterpret_cast<int*>(&_meshTriIndices[0]);
	int indexCount = sizeof(IndexedTriangle) * _meshTriIndices.size();
	meshIB.allocate(indices, indexCount);

	meshVAO.release();	

	reset_bounding_box();
	for (auto& v : _meshVertices)
		add_point(v.x(), v.y(), v.z());

	emit meshChanged();
}

void CaveGLData::drawCave()
{
	if (_meshTriIndices.size() == 0)
		return;

	glm::mat4 modelView, proj;
	glGetFloatv(GL_MODELVIEW_MATRIX, glm::value_ptr(modelView));
	glGetFloatv(GL_PROJECTION_MATRIX, glm::value_ptr(proj));
	auto MVP = glm::transpose(proj * modelView);

	program->bind();
	meshVAO.bind();	

	auto m = QMatrix4x4(glm::value_ptr(MVP));
	program->setUniformValue("mvp", m);

	glDrawElements(GL_TRIANGLES, _meshTriIndices.size() * 3, GL_UNSIGNED_INT, 0);

	program->release();
	meshVAO.release();
}

void CaveGLData::initGL(const QGLContext * ctx)
{	
	initializeGLFunctions(ctx);

	if (program == nullptr)
		init_shaders(ctx);

	meshVB.create();
	meshIB.create();
}

void CaveGLData::init_shaders(const QGLContext * context)
{
	program = std::make_unique<QGLShaderProgram>(context);

	bool success;
	success = program->addShaderFromSourceFile(QGLShader::Vertex, "cave.vert");
	if (!success)
		QMessageBox::critical(nullptr, "Vertex Shader", program->log());

	success = program->addShaderFromSourceFile(QGLShader::Geometry, "cave.geom");
	if (!success)
		QMessageBox::critical(nullptr, "Geometry Shader", program->log());

	success = program->addShaderFromSourceFile(QGLShader::Fragment, "cave.frag");
	if (!success)
		QMessageBox::critical(nullptr, "Fragment Shader", program->log());

	success = program->link();
	if (!success)
		QMessageBox::critical(nullptr, "Shaderkompilierung", program->log());
}
