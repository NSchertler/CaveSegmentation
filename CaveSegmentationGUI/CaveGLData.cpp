#include "stdafx.h"

#include "CaveGLData.hpp"
#include "GLUtils.h"
#include <qmessagebox.h>
#include <glm/gtc/type_ptr.hpp>

#include <set>
#include <queue>

std::unique_ptr<QOpenGLShaderProgram> CaveGLData::meshProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonPointProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::skeletonPointSelectionProgram(nullptr);
std::unique_ptr<QOpenGLShaderProgram> CaveGLData::correspondenceProgram(nullptr);

CaveGLData::CaveGLData(QObject * parent)
	: QObject(parent), meshVB(QOpenGLBuffer::VertexBuffer), meshIB(QOpenGLBuffer::IndexBuffer),
	  skeletonVB(QOpenGLBuffer::VertexBuffer), skeletonIB(QOpenGLBuffer::IndexBuffer), correspondenceVB(QOpenGLBuffer::VertexBuffer), colorLayerVBO(QOpenGLBuffer::VertexBuffer)
{		
}

CaveGLData::~CaveGLData()
{
	meshVB.destroy();
	meshIB.destroy();

	skeletonVB.destroy();
	skeletonIB.destroy();
}

void CaveGLData::LoadMesh(const std::string & offFile)
{
	CaveData::LoadMesh(offFile);	

	ctx->MakeOpenGLContextCurrent();
	meshVAO.create();
	meshVAO.bind();

	meshVB.create();
	meshVB.bind();
	std::vector<glm::vec4> positions(_meshVertices.size());
	for(int i = 0; i < _meshVertices.size(); ++i)
		positions[i] = glm::vec4(_meshVertices.at(i).x(), _meshVertices.at(i).y(), _meshVertices.at(i).z(), 1);

	meshVB.allocate(&positions[0], sizeof(glm::vec4) * positions.size());
	meshProgram->enableAttributeArray("in_position");
	meshProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);
	
	meshIB.create();
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

void CaveGLData::drawCave(CameraProvider* cam)
{
	if (!meshVAO.isCreated())
		return;

	meshVAO.bind();
	meshProgram->bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));
	meshProgram->setUniformValue("mvp", m);

	glDrawElements(GL_TRIANGLES, _meshTriIndices.size() * 3, GL_UNSIGNED_INT, 0);

	meshProgram->release();
	meshVAO.release();
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
	CaveData::SetSkeleton(skeleton);

	ctx->MakeOpenGLContextCurrent();
	skeletonVAO.create();
	skeletonVAO.bind();

	skeletonVB.create();
	skeletonVB.bind();
	std::vector<glm::vec4> skeletonPositions(skeleton->vertices.size());
	for (size_t i = 0; i < skeleton->vertices.size(); ++i)
		skeletonPositions.at(i) = glm::vec4(skeleton->vertices.at(i).position.x(), skeleton->vertices.at(i).position.y(), skeleton->vertices.at(i).position.z(), 1);
	skeletonVB.allocate(&skeletonPositions[0], sizeof(glm::vec4) * skeletonPositions.size());
	skeletonProgram->enableAttributeArray("in_position");
	skeletonProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);

	skeletonIB.create();
	skeletonIB.bind();
	int* indices = reinterpret_cast<int*>(&skeleton->edges[0]);
	int indexCount = sizeof(std::pair<int, int>) * skeleton->edges.size();
	skeletonIB.allocate(indices, indexCount);	

	//color layer
	colorLayer.resize(skeleton->vertices.size(), glm::vec4(1, 1, 1, 1));
	colorLayerVBO.create();
	colorLayerVBO.bind();
	colorLayerVBO.allocate(&colorLayer[0], sizeof(glm::vec4) * colorLayer.size());
	skeletonPointProgram->enableAttributeArray("colorLayer");
	skeletonPointProgram->setAttributeBuffer("colorLayer", GL_FLOAT, 0, 4);

	skeletonVAO.release();

	//Create correspondences
	correspondenceVAO.create();
	correspondenceVAO.bind();

	meshVB.bind();
	correspondenceProgram->enableAttributeArray("in_position");
	skeletonProgram->setAttributeBuffer("in_position", GL_FLOAT, 0, 4);

	uint32_t* correspondingSkeletonVertices = new uint32_t[_meshVertices.size()];
	for (int v = 0; v < skeleton->vertices.size(); ++v)
		for (int corr : skeleton->vertices.at(v).correspondingOriginalVertices)
			correspondingSkeletonVertices[corr] = v;
	correspondenceVB.create();
	correspondenceVB.bind();
	correspondenceVB.allocate(correspondingSkeletonVertices, _meshVertices.size() * sizeof(uint32_t));
	correspondenceProgram->enableAttributeArray("in_correspondence");
	glVertexAttribIPointer(correspondenceProgram->attributeLocation("in_correspondence"), 1, GL_UNSIGNED_INT, 0, 0);

	delete[] correspondingSkeletonVertices;

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, skeletonVB.bufferId());

	correspondenceVAO.release();	

	emit skeletonChanged();
}

void CaveGLData::drawSkeleton(CameraProvider* cam)
{
	if (!skeletonVAO.isCreated())
		return;

	skeletonVAO.bind();
	skeletonProgram->bind();
	
	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));
	skeletonProgram->setUniformValue("mvp", m);
	glDrawElements(GL_LINES, skeleton->edges.size() * 2, GL_UNSIGNED_INT, 0);

	skeletonPointProgram->bind();
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	skeletonPointProgram->setUniformValue("mvp", m);
	skeletonPointProgram->setUniformValue("viewportDiagonal", sqrtf(viewport[2] * viewport[2] + viewport[3] * viewport[3]));
	skeletonPointProgram->setUniformValue("sizeMultiplier", 0.5f);
		
	glDrawArrays(GL_POINTS, 0, skeleton->vertices.size());

	skeletonPointProgram->release();
	skeletonVAO.release();
}

void CaveGLData::drawSelectedSkeletonVertex(CameraProvider* cam, int selected)
{
	if (!skeletonVAO.isCreated())
		return;

	skeletonVAO.bind();
	
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
	skeletonVAO.release();
}

void CaveGLData::drawCorrespondence(CameraProvider* cam, int specificSkeletonVertex)
{
	if (!correspondenceVAO.isCreated())
		return;

	correspondenceVAO.bind();
	correspondenceProgram->bind();

	auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
	auto m = QMatrix4x4(glm::value_ptr(MVP));

	correspondenceProgram->setUniformValue("mvp", m);	
	correspondenceProgram->setUniformValue("specificSkeletonVertex", specificSkeletonVertex);
	glDrawArrays(GL_POINTS, 0, _meshVertices.size());

	correspondenceProgram->release();
	correspondenceVAO.release();
}

void CaveGLData::initGL(OpenGLContextProvider* ctx)
{	
	this->ctx = ctx;

	initializeOpenGLFunctions();

	if (meshProgram == nullptr)
		init_shaders();
}

void CaveGLData::init_shaders()
{
	meshProgram = MakeProgram("cave.vert", "cave.frag", "cave.geom");
	skeletonProgram = MakeProgram("skeleton.vert", "skeleton.frag");
	skeletonPointProgram = MakeProgram("skeletonPoint.vert", "skeletonPoint.frag");
	skeletonPointSelectionProgram = MakeProgram("skeletonPoint.vert", "skeletonPointSelection.frag");
	correspondenceProgram = MakeProgram("correspondence.vert", "correspondence.frag", "correspondence.geom");
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

	openSet.emplace(startVertex, 0.0, (skeleton->vertices.at(targetVertex).position - skeleton->vertices.at(startVertex).position).norm());

	while (!openSet.empty())
	{
		Node active = openSet.top();
		openSet.pop();

		if (active.vertex == targetVertex)
			break;
		if (closedSet.find(active.vertex) != closedSet.end())
			continue;

		closedSet.insert(closedSet.end(), active.vertex);
		for (int neighbor : adjacency.at(active.vertex))
		{
			if (closedSet.find(neighbor) != closedSet.end())
				continue;

			double cost = active.costSoFar + (skeleton->vertices.at(active.vertex).position - skeleton->vertices.at(neighbor).position).norm();
			auto minCostEntry = minCost.find(neighbor);
			if (minCostEntry == minCost.end() || cost < minCostEntry->second)
			{
				minCost[neighbor] = cost;
				predecessors[neighbor] = active.vertex;
				openSet.emplace(neighbor, cost, (skeleton->vertices.at(targetVertex).position - skeleton->vertices.at(neighbor).position).norm());
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