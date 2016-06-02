#pragma once
#include <QObject>
#include <QOpenGLFunctions_3_3_Core>
#include <qopenglbuffer.h>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <deque>

#include "CaveData.h"
#include "IHasBoundingBox.h"
#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

class CaveGLData : public QObject, public CaveData, QOpenGLFunctions_3_3_Core, public IHasBoundingBox {
	Q_OBJECT

public:
	CaveGLData(QObject * parent = Q_NULLPTR);
	~CaveGLData();

	virtual void LoadMesh(const std::string& offFile);
	virtual void SetSkeleton(CurveSkeleton* skeleton);
	virtual void SmoothAndDeriveDistances();

	void drawCave(CameraProvider*);
	void ResetColorLayer();
	void UpdateColorLayer();
	void drawSkeleton(CameraProvider*);

	void drawSkeletonPoints(CameraProvider * cam);

	void drawSelectedSkeletonVertex(CameraProvider * cam, int selected);

	void drawCorrespondence(CameraProvider * cam, int specificSkeletonVertex = -1);

	void initGL(OpenGLContextProvider*, bool primary);
	static void init_shaders();

	void FindPath(int startVertex, int targetVertex, std::deque<int>& resultPath);

	std::vector<glm::vec4> colorLayer;	

	std::vector<int32_t> segmentation;
	void RunSegmentation();
	void LoadSegmentation(const std::string & path);	

signals:
	void meshChanged();
	void skeletonChanged();
	void distancesChanged();
	void segmentationChanged();

private:
	glm::vec4 segmentColor(int32_t segment);
	void segmentationChangedHandler();

	QOpenGLBuffer meshVB, meshIB, meshSegColor;	
	QOpenGLBuffer skeletonVB, skeletonIB, correspondenceVB, colorLayerVBO, skeletonSegColor;	

	struct ContextSpecificData
	{
		QOpenGLVertexArrayObject meshVAO;
		QOpenGLVertexArrayObject skeletonVAO, correspondenceVAO;

		OpenGLContextProvider* provider;
	};

	std::map<QOpenGLContext*, ContextSpecificData> contextSpecificData;

	OpenGLContextProvider* ctx;	

	static std::unique_ptr<QOpenGLShaderProgram> meshProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonProgram;
	static std::unique_ptr<QOpenGLShaderProgram> correspondenceProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonPointProgram;
	static std::unique_ptr<QOpenGLShaderProgram> skeletonPointSelectionProgram;
};
