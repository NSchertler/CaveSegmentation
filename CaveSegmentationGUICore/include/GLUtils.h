#pragma once

#include "cavesegmentationguicore_global.h"

#include <memory>
#include <QString>
#include <QOpenGLShaderProgram>

std::unique_ptr<QOpenGLShaderProgram> CAVESEGMENTATIONGUICORE_EXPORT MakeProgram(QString vsPath, QString fsPath, QString gsPath = QString());