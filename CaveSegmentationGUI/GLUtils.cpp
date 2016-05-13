#include "stdafx.h"
#include "GLUtils.h"

#include <QMessageBox>

std::unique_ptr<QOpenGLShaderProgram> MakeProgram(QString vsPath, QString fsPath, QString gsPath)
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
		QMessageBox::critical(nullptr, "Shader Linking Error", program->log());
		return nullptr;
	}

	return std::move(program);
}