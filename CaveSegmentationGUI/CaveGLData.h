#pragma once
#include <QObject>
#include <QOpenGLFunctions_3_3_Core>
#include <qopenglbuffer.h>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <deque>
#include <memory>

#include "ICaveData.h"
#include "IHasBoundingBox.h"
#include "CameraProvider.h"
#include "OpenGLContextProvider.h"

//Wrapper of CaveData that adds OpenGL-specific content and additional functionality.
class CaveGLData : public QObject, public ICaveData
{
	Q_OBJECT

public:
	CaveGLData(QObject * parent = Q_NULLPTR);
	~CaveGLData();

	//Overridden functions from decorated ICaveData
	void LoadMesh(const std::string& offFile);
	void SetSkeleton(CurveSkeleton* skeleton);
	void SmoothAndDeriveDistances();

	//ICaveData forwarded functions
	void WriteMesh(const std::string& offFile, std::function<void(int i, int& r, int& g, int& b)> colorFunc) const { decoratee->WriteMesh(offFile, colorFunc); }
	void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation) const { decoratee->WriteSegmentationColoredOff(path, segmentation); }
	bool CalculateDistances(float exponent = 1.0f) { return decoratee->CalculateDistances(exponent); }
	bool CalculateDistancesSingleVertexWithDebugOutput(int iVert, float exponent = 1.0f) { return decoratee->CalculateDistancesSingleVertexWithDebugOutput(iVert, exponent); }
	void LoadDistances(const std::string& file) { decoratee->LoadDistances(file); }
	void SaveDistances(const std::string& file) const { decoratee->SaveDistances(file); }
	void SetOutputDirectory(const std::wstring& outputDirectory) { decoratee->SetOutputDirectory(outputDirectory); }
	void ResizeMeshAttributes(size_t vertexCount) { decoratee->ResizeMeshAttributes(vertexCount); }
	void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount) { decoratee->ResizeSkeletonAttributes(vertexCount, edgeCount); }
	size_t MeshVertexCorrespondsTo(size_t meshVertex) const { return decoratee->MeshVertexCorrespondsTo(meshVertex); }
	const CurveSkeleton* Skeleton() const { return decoratee->Skeleton(); }
	ICaveData::Algorithm& CaveScaleAlgorithm() { return decoratee->CaveScaleAlgorithm(); }
	double& CaveScaleKernelFactor() { return decoratee->CaveScaleKernelFactor(); }
	double& CaveSizeKernelFactor() { return decoratee->CaveSizeKernelFactor(); }
	double& CaveSizeDerivativeKernelFactor() { return decoratee->CaveSizeDerivativeKernelFactor(); }
	bool HasCaveSizes() const { return decoratee->HasCaveSizes(); }
	bool HasUnsmoothedCaveSizes() const { return decoratee->HasUnsmoothedCaveSizes(); }
	const std::vector<size_t>& VerticesWithInvalidSize() const { return decoratee->VerticesWithInvalidSize(); }
	double CaveSize(size_t iVertex) const { return decoratee->CaveSize(iVertex); }
	double CaveSizeUnsmoothed(size_t iVertex) const { return decoratee->CaveSizeUnsmoothed(iVertex); }
	double CaveScale(size_t iVertex) const { return decoratee->CaveScale(iVertex); }
	double CaveSizeDerivative(size_t iEdge) const { return decoratee->CaveSizeDerivative(iEdge); }
	double CaveSizeCurvature(size_t iEdge) const { return decoratee->CaveSizeCurvature(iEdge); }
	void SetVerbose(bool verbose) { decoratee->SetVerbose(verbose); }

	//IGraph forwarded functions
	const std::vector<int>& AdjacentNodes(size_t skeletonVertex) const { return decoratee->AdjacentNodes(skeletonVertex); }
	size_t EdgeIdFromVertexPair(size_t v1, size_t v2) const { return decoratee->EdgeIdFromVertexPair(v1, v2); }
	void IncidentVertices(size_t edgeId, size_t& v1, size_t& v2) const { decoratee->IncidentVertices(edgeId, v1, v2); }
	const Eigen::Vector3f& VertexPosition(size_t vertexId) const { return decoratee->VertexPosition(vertexId); }
	double NodeRadius(size_t vertexId) const { return decoratee->NodeRadius(vertexId); }
	size_t NumberOfVertices() const { return decoratee->NumberOfVertices(); }
	size_t NumberOfEdges() const { return decoratee->NumberOfEdges(); }

	//IMesh forwarded functions
	const std::vector<Eigen::Vector3f>& MeshVertices() const { return decoratee->MeshVertices(); }
	const std::vector<IndexedTriangle>& MeshTriIndices() const { return decoratee->MeshTriIndices(); }

	//IHasBoundingBox forwarded functions
	const Eigen::Vector3f& GetMin() const { return decoratee->GetMin(); }
	const Eigen::Vector3f& GetMax() const { return decoratee->GetMax(); }

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
	std::shared_ptr<ICaveData> decoratee;
	QOpenGLFunctions_3_3_Core gl;

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
