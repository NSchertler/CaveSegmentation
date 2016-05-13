#pragma once
#include <QObject>
#include <QOpenGLFunctions_4_3_Core>
#include <qopenglbuffer.h>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <deque>

#include "CaveData.h"
#include "IHasBoundingBox.h"
#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

class CaveGLData : public QObject, public CaveData, QOpenGLFunctions_4_3_Core, public IHasBoundingBox {
	Q_OBJECT

public:
	CaveGLData(QObject * parent = Q_NULLPTR);
	~CaveGLData();

	virtual void LoadMesh(const std::string& offFile);
	virtual void SetSkeleton(CurveSkeleton* skeleton);

	void drawCave(CameraProvider*);
	void ResetColorLayer();
	void UpdateColorLayer();
	void drawSkeleton(CameraProvider*);

	void drawSelectedSkeletonVertex(CameraProvider * cam, int selected);

	void drawCorrespondence(CameraProvider * cam, int specificSkeletonVertex = -1);

	void initGL(OpenGLContextProvider*);
	static void init_shaders();

	void FindPath(int startVertex, int targetVertex, std::deque<int>& resultPath);

	std::vector<glm::vec4> colorLayer;

signals:
	void meshChanged();
	void skeletonChanged();

private:
	QOpenGLBuffer meshVB, meshIB;
	QOpenGLVertexArrayObject meshVAO;

	QOpenGLBuffer skeletonVB, skeletonIB, correspondenceVB, colorLayerVBO;
	QOpenGLVertexArrayObject skeletonVAO, correspondenceVAO;

	OpenGLContextProvider* ctx;	

	static std::unique_ptr<QOpenGLShaderProgram> meshProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonProgram;
	static std::unique_ptr<QOpenGLShaderProgram> correspondenceProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonPointProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonPointSelectionProgram;
};
