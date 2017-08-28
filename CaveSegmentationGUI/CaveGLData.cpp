#include "stdafx.h"

#include "CaveGLData.h"
#include <GLUtils.h>
#include <qmessagebox.h>
#include <QDir>
#include <glm/gtc/type_ptr.hpp>

#include <set>
#include <queue>
#include <fstream>

#include <ChamberAnalyzation/CurvatureBasedQPBO.h>
#include <ChamberAnalyzation/Utils.h>
#include <FileInputOutput.h>

std::unique_ptr<QOpenGLShaderProgram> CaveGLData::meshProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonPointProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonPointSelectionProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::correspondenceProgram(nullptr);

CaveGLData::CaveGLData(QObject * parent)
	: QObject(parent), meshVB(QOpenGLBuffer::VertexBuffer), meshIB(QOpenGLBuffer::IndexBuffer), meshSegColor(QOpenGLBuffer::VertexBuffer), 
	  skeletonVB(QOpenGLBuffer::VertexBuffer), skeletonIB(QOpenGLBuffer::IndexBuffer), correspondenceVB(QOpenGLBuffer::VertexBuffer), colorLayerVBO(QOpenGLBuffer::VertexBuffer), skeletonSegColor(QOpenGLBuffer::VertexBuffer)
{
	decoratee = CreateCaveData();
	connect(this, &CaveGLData::segmentationChanged, this, &CaveGLData::segmentationChangedHandler);
}

CaveGLData::~CaveGLData()
{
}

void CaveGLData::RunSegmentation()
{
	CurvatureBasedQPBO::FindChambers(*this, segmentation);
	AssignUniqueChamberIndices(*this, segmentation);

	emit segmentationChanged();
}

void CaveGLData::LoadSegmentation(const std::string& path)
{
	ReadSegmentation(path, segmentation, NumberOfVertices());
	AssignUniqueChamberIndices(*this, segmentation);

	emit segmentationChanged();
}

glm::vec4 CaveGLData::segmentColor(int32_t segment)
{
	const int* c = GetSegmentColor(segment);
	return glm::vec4(c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f, 1);
}

void CaveGLData::segmentationChangedHandler()
{
	std::vector<glm::vec4> meshSegColors(MeshVertices().size());
	for (int i = 0; i < MeshVertices().size(); ++i)
	{
		int segment = segmentation.at(MeshVertexCorrespondsTo(i));
		meshSegColors.at(i) = segmentColor(segment);
	}
	meshSegColor.bind();
	meshSegColor.write(0, &meshSegColors[0], sizeof(glm::vec4) * meshSegColors.size());

	std::vector<glm::vec4> skelSegColors(NumberOfVertices());
	for (int i = 0; i < NumberOfVertices(); ++i)
	{
		int segment = segmentation.at(i);
		skelSegColors.at(i) = segmentColor(segment);
	}
	skeletonSegColor.bind();
	skeletonSegColor.write(0, &skelSegColors[0], sizeof(glm::vec4) * skelSegColors.size());
}


void CaveGLData::LoadMesh(const std::string & offFile)
{
	decoratee->LoadMesh(offFile);

	QDir outDir(QString::fromStdString(offFile));
	outDir.cdUp();
	outDir.mkdir("output");
	outDir.cd("output");
	SetOutputDirectory(outDir.path().toStdWString());

	SetSkeleton(nullptr);
	segmentation.resize(0);

	ctx->MakeOpenGLContextCurrent();

	meshVB.create();
	meshVB.bind();
	std::vector<glm::vec4> positions(MeshVertices().size());
	for(int i = 0; i < MeshVertices().size(); ++i)
		positions[i] = glm::vec4(MeshVertices().at(i).x(), MeshVertices().at(i).y(), MeshVertices().at(i).z(), 1);

	meshVB.allocate(&positions[0], sizeof(glm::vec4) * positions.size());
	
	meshSegColor.create();
	meshSegColor.bind();
	meshSegColor.allocate(sizeof(glm::vec4) * positions.size());	
	
	meshIB.create();
	meshIB.bind();
	const int* indices = reinterpret_cast<const int*>(MeshTriIndices().data());
	int indexCount = sizeof(IndexedTriangle) * MeshTriIndices().size();
	meshIB.allocate(indices, indexCount);

	for (auto& c : contextSpecificData)
	{
		c.second.provider->MakeOpenGLContextCurrent();
		c.second.meshVAO.create();
		c.second.meshVAO.bind();

		meshVB.bind();
		meshProgram->enableAttributeArray("in_position");
		meshProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);

		meshSegColor.bind();
		meshProgram->enableAttributeArray("segColor");
		meshProgram->setAttributeBuffer("segColor", GL_FLOAT, 0, 4);

		meshIB.bind();

		c.second.meshVAO.release();
	}
	
	emit meshChanged();
}

void CaveGLData::drawCave(CameraProvider* cam)
{
	ContextSpecificData& data = contextSpecificData.at(QOpenGLContext::currentContext());

	if (!data.meshVAO.isCreated())
		return;

	data.meshVAO.bind();
	bool result = meshProgram->bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto mvp = QMatrix4x4(glm::value_ptr(MVP));	
	meshProgram->setUniformValue("mvp", mvp);

	auto MV = glm::transpose(cam->GetViewMatrix());
	auto mv = QMatrix4x4(glm::value_ptr(MV));
	meshProgram->setUniformValue("mv", mv);

	meshProgram->setUniformValue("hasSegmentation", segmentation.size() == 0 ? 0 : 1);

	glDrawElements(GL_TRIANGLES, MeshTriIndices().size() * 3, GL_UNSIGNED_INT, 0);

	meshProgram->release();
	data.meshVAO.release();
}

void CaveGLData::ResetColorLayer()
{
	for (int i = 0; i < colorLayer.size(); ++i)
		colorLayer.at(i) = glm::vec4(1, 1, 1, 1);
}

void CaveGLData::UpdateColorLayer()
{
	ctx->MakeOpenGLContextCurrent();
	colorLayerVBO.bind();
	colorLayerVBO.write(0, &colorLayer[0], sizeof(glm::vec4) * colorLayer.size());
}

void CaveGLData::SetSkeleton(CurveSkeleton * skeleton)
{
	decoratee->SetSkeleton(skeleton);	
	if (skeleton)
	{
		ctx->MakeOpenGLContextCurrent();		

		skeletonVB.create();
		skeletonVB.bind();
		std::vector<glm::vec4> skeletonPositions(skeleton->vertices.size());
		for (size_t i = 0; i < skeleton->vertices.size(); ++i)
			skeletonPositions.at(i) = glm::vec4(skeleton->vertices.at(i).position.x(), skeleton->vertices.at(i).position.y(), skeleton->vertices.at(i).position.z(), 1);
		skeletonVB.allocate(&skeletonPositions[0], sizeof(glm::vec4) * skeletonPositions.size());		

		//color layer
		colorLayer.resize(skeleton->vertices.size(), glm::vec4(1, 1, 1, 1));
		colorLayerVBO.create();
		colorLayerVBO.bind();
		colorLayerVBO.allocate(&colorLayer[0], sizeof(glm::vec4) * colorLayer.size());		

		skeletonSegColor.create();
		skeletonSegColor.bind();
		skeletonSegColor.allocate(sizeof(glm::vec4) * colorLayer.size());		

		skeletonIB.create();
		skeletonIB.bind();
		int* indices = reinterpret_cast<int*>(&skeleton->edges[0]);
		int indexCount = sizeof(std::pair<int, int>) * skeleton->edges.size();
		skeletonIB.allocate(indices, indexCount);

		for (auto& c : contextSpecificData)
		{
			c.second.provider->MakeOpenGLContextCurrent();
			c.second.skeletonVAO.create();
			c.second.skeletonVAO.bind();

			skeletonVB.bind();
			skeletonProgram->enableAttributeArray("in_position");
			skeletonProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);

			colorLayerVBO.bind();
			skeletonPointProgram->enableAttributeArray("colorLayer");
			skeletonPointProgram->setAttributeBuffer("colorLayer", GL_FLOAT, 0, 4);

			skeletonSegColor.bind();
			skeletonPointProgram->enableAttributeArray("segColor");
			skeletonPointProgram->setAttributeBuffer("segColor", GL_FLOAT, 0, 4);

			skeletonIB.bind();

			c.second.skeletonVAO.release();
		}

		if (correspondenceProgram != nullptr)
		{
			//Create correspondences											

			uint32_t* correspondingSkeletonVertices = new uint32_t[MeshVertices().size()];
			for (int v = 0; v < skeleton->vertices.size(); ++v)
				for (int corr : skeleton->vertices.at(v).correspondingOriginalVertices)
					correspondingSkeletonVertices[corr] = v;
			correspondenceVB.create();
			correspondenceVB.bind();
			correspondenceVB.allocate(correspondingSkeletonVertices, MeshVertices().size() * sizeof(uint32_t));			

			delete[] correspondingSkeletonVertices;

			for (auto& c : contextSpecificData)
			{
				c.second.provider->MakeOpenGLContextCurrent();
				c.second.correspondenceVAO.create();
				c.second.correspondenceVAO.bind();

				meshVB.bind();
				correspondenceProgram->enableAttributeArray("in_position");
				skeletonProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);

				correspondenceVB.bind();
				correspondenceProgram->enableAttributeArray("in_correspondence");
				gl.glVertexAttribIPointer(correspondenceProgram->attributeLocation("in_correspondence"), 1, GL_UNSIGNED_INT, 0, 0);

				gl.glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, skeletonVB.bufferId());

				c.second.correspondenceVAO.release();
			}			
		}
	}
	else
	{
		for (auto& c : contextSpecificData)
		{
			c.second.provider->MakeOpenGLContextCurrent();
			c.second.skeletonVAO.destroy();
		}
	}

	emit skeletonChanged();
	emit distancesChanged();
}

void CaveGLData::SmoothAndDeriveDistances()
{	
	if (HasUnsmoothedCaveSizes() && CaveSizeUnsmoothed(0) > 0)
	{
		decoratee->SmoothAndDeriveDistances();
		emit distancesChanged();
	}
}

void CaveGLData::drawSkeleton(CameraProvider* cam)
{
	ContextSpecificData& data = contextSpecificData.at(QOpenGLContext::currentContext());

	if (!data.skeletonVAO.isCreated())
		return;

	data.skeletonVAO.bind();
	skeletonProgram->bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));
	skeletonProgram->setUniformValue("mvp", m);

	glDrawElements(GL_LINES, NumberOfEdges() * 2, GL_UNSIGNED_INT, 0);

	skeletonProgram->release();
	data.skeletonVAO.release();
}

void CaveGLData::drawSkeletonPoints(CameraProvider* cam)
{
	ContextSpecificData& data = contextSpecificData.at(QOpenGLContext::currentContext());

	if (!data.skeletonVAO.isCreated())
		return;

	data.skeletonVAO.bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));

	skeletonPointProgram->bind();
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	skeletonPointProgram->setUniformValue("mvp", m);
	skeletonPointProgram->setUniformValue("viewportDiagonal", sqrtf(viewport[2] * viewport[2] + viewport[3] * viewport[3]));
	skeletonPointProgram->setUniformValue("sizeMultiplier", 0.5f);
	skeletonPointProgram->setUniformValue("hasSegmentation", static_cast<GLuint>(segmentation.size() > 0));

	glDrawArrays(GL_POINTS, 0, NumberOfVertices());

	skeletonPointProgram->release();
	data.skeletonVAO.release();
}

void CaveGLData::drawSelectedSkeletonVertex(CameraProvider* cam, int selected)
{
	ContextSpecificData& data = contextSpecificData.at(QOpenGLContext::currentContext());

	if (!data.skeletonVAO.isCreated())
		return;

	data.skeletonVAO.bind();
	
	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));
	
	skeletonPointSelectionProgram->bind();
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	skeletonPointSelectionProgram->setUniformValue("mvp", m);
	skeletonPointSelectionProgram->setUniformValue("viewportDiagonal", sqrtf(viewport[2] * viewport[2] + viewport[3] * viewport[3]));
	skeletonPointSelectionProgram->setUniformValue("sizeMultiplier", 0.7f);

	glDrawArrays(GL_POINTS, selected, 1);

	skeletonPointSelectionProgram->release();
	data.skeletonVAO.release();
}

void CaveGLData::drawCorrespondence(CameraProvider* cam, int specificSkeletonVertex)
{
	ContextSpecificData& data = contextSpecificData.at(QOpenGLContext::currentContext());

	if (!data.correspondenceVAO.isCreated() || correspondenceProgram == nullptr)
		return;

	data.correspondenceVAO.bind();
	correspondenceProgram->bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));

	correspondenceProgram->setUniformValue("mvp", m);	
	correspondenceProgram->setUniformValue("specificSkeletonVertex", specificSkeletonVertex);
	glDrawArrays(GL_POINTS, 0, MeshVertices().size());

	correspondenceProgram->release();
	data.correspondenceVAO.release();
}

void CaveGLData::initGL(OpenGLContextProvider* ctx, bool primary)
{	
	if (primary)
	{
		this->ctx = ctx;
		gl.initializeOpenGLFunctions();
	}

	if (meshProgram == nullptr)
		init_shaders();

	contextSpecificData[QOpenGLContext::currentContext()].provider = ctx;
}

void CaveGLData::init_shaders()
{
	meshProgram = MakeProgram(":/glsl/cave.vert", ":/glsl/cave.frag", ":/glsl/cave.geom");
	skeletonProgram = MakeProgram(":/glsl/skeleton.vert", ":/glsl/skeleton.frag", ":/glsl/skeleton.geom");
	skeletonPointProgram = MakeProgram(":/glsl/skeletonPoint.vert", ":/glsl/skeletonPoint.frag");
	skeletonPointSelectionProgram = MakeProgram(":/glsl/skeletonPoint.vert", ":/glsl/skeletonPointSelection.frag");
	correspondenceProgram = MakeProgram(":/glsl/correspondence.vert", ":/glsl/correspondence.frag", ":/glsl/correspondence.geom");
}

void CaveGLData::FindPath(int startVertex, int targetVertex, std::deque<int>& resultPath)
{
	//A* search

	struct Node
	{
		int vertex;
		double costSoFar;
		double estimatedCostToTarget;

		Node(int vertex, double costSoFar, double estimatedCostToTarget)
			: vertex(vertex), costSoFar(costSoFar), estimatedCostToTarget(estimatedCostToTarget)
		{ }

		bool operator<(const Node& rhs) const
		{
			return costSoFar + estimatedCostToTarget > rhs.costSoFar + rhs.estimatedCostToTarget;
		}
	};

	
	std::priority_queue<Node> openSet;
	std::set<int> closedSet;

	std::map<int, int> predecessors;
	std::map<int, double> minCost;
	predecessors[startVertex] = -1;
	minCost[startVertex] = 0;

	openSet.emplace(startVertex, 0.0, (VertexPosition(targetVertex) - VertexPosition(startVertex)).norm());

	while (!openSet.empty())
	{
		Node active = openSet.top();
		openSet.pop();

		if (active.vertex == targetVertex)
			break;
		if (closedSet.find(active.vertex) != closedSet.end())
			continue;

		closedSet.insert(closedSet.end(), active.vertex);
		for (int neighbor : AdjacentNodes(active.vertex))
		{
			if (closedSet.find(neighbor) != closedSet.end())
				continue;

			double cost = active.costSoFar + (VertexPosition(active.vertex) - VertexPosition(neighbor)).norm();
			auto minCostEntry = minCost.find(neighbor);
			if (minCostEntry == minCost.end() || cost < minCostEntry->second)
			{
				minCost[neighbor] = cost;
				predecessors[neighbor] = active.vertex;
				openSet.emplace(neighbor, cost, (VertexPosition(targetVertex) - VertexPosition(neighbor)).norm());
			}
		}
	}

	int v = targetVertex;
	while (v != -1)
	{
		resultPath.push_front(v);
		v = predecessors.at(v);
	}
}

void CaveGLData::WriteBIN(const std::string& filename) const
{
	std::ofstream binaryOutput(filename, std::ios::binary);

	int nVertices = MeshVertices().size();
	int nIndices = 3 * MeshTriIndices().size();
	binaryOutput.write(reinterpret_cast<const char*>(&nVertices), sizeof(int));
	binaryOutput.write(reinterpret_cast<const char*>(&nIndices), sizeof(int));
	binaryOutput.write(reinterpret_cast<const char*>(MeshVertices().data()), nVertices * sizeof(Eigen::Vector3f));
	binaryOutput.write(reinterpret_cast<const char*>(MeshTriIndices().data()), MeshTriIndices().size() * sizeof(IndexedTriangle));

	binaryOutput.close();
}