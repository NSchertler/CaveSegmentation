#pragma once

#include <memory>
#include <QString>
#include <QOpenGLShaderProgram>

std::unique_ptr<QOpenGLShaderProgram> MakeProgram(QString vsPath, QString fsPath, QString gsPath = QString());