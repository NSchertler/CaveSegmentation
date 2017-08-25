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

//Wrapper of CaveData that adds OpenGL-specific content and additional functionality.
class CaveGLData : public QObject, public CaveData, QOpenGLFunctions_3_3_Core
{
	Q_OBJECT

public:
	CaveGLData(QObject * parent = Q_NULLPTR);
	~CaveGLData();

	virtual void LoadMesh(const std::string& offFile);
	virtual void SetSkeleton(CurveSkeleton* skeleton);
	virtual void SmoothAndDeriveDistances();

	//Rendering function for the cave
	void drawCave(CameraProvider*);

	//Rendering function for the skeleton lines
	void drawSkeleton(CameraProvider*);

	//Rendering function for the skeleton nodes
	void drawSkeletonPoints(CameraProvider * cam);

	//Rendering function for the selected skeleton node
	void drawSelectedSkeletonVertex(CameraProvider * cam, int selected);

	//Rendering function for the skeleton-to-cave correspondences
	void drawCorrespondence(CameraProvider * cam, int specificSkeletonVertex = -1);

	//Resets the multiplicative color layer to the identity color (white).
	void ResetColorLayer();
	//Sends the current color layer to the GPU.
	void UpdateColorLayer();		

	void initGL(OpenGLContextProvider*, bool primary);
	static void init_shaders();

	//Use A* to find the shortest path in the skeleton from startVertex to targetVertex
	void FindPath(int startVertex, int targetVertex, std::deque<int>& resultPath);

	//A multiplicative color layer for the skeleton nodes.
	std::vector<glm::vec4> colorLayer;	

	//Segmentation indices for the skeleton nodes
	std::vector<int32_t> segmentation;

	//Calculate the segmentation based on the current data.
	void RunSegmentation();

	//Load a segmentation from file
	void LoadSegmentation(const std::string & path);	

	//Writes a binary file suitable for manual cave segmentation
	void WriteBIN(const std::string& filename) const;
signals:
	void meshChanged();
	void skeletonChanged();
	void distancesChanged();
	void segmentationChanged();

private:
	//Returns a color for a given segment number
	glm::vec4 segmentColor(int32_t segment);
	void segmentationChangedHandler();

	QOpenGLBuffer meshVB, meshIB, meshSegColor;	
	QOpenGLBuffer skeletonVB, skeletonIB, correspondenceVB, colorLayerVBO, skeletonSegColor;	

	//If stereo rendering is used, we have multiple contexts and, thus, need to maintain multiple VAOs.
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
