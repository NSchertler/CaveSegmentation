#include "stdafx.h"

#include "CaveGLData.hpp"
#include "GLUtils.h"
#include <qmessagebox.h>
#include <glm/gtc/type_ptr.hpp>

#include <set>
#include <queue>

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
	ReadSegmentation(path, segmentation, skeleton);
	AssignUniqueChamberIndices(*this, segmentation);

	emit segmentationChanged();
}

glm::vec4 CaveGLData::segmentColor(int32_t segment)
{
	const glm::vec4 colors[10] =
	{
		glm::vec4( 166.0f / 255.0f, 206.0f / 255.0f, 227.0f / 255.0f, 1 ),
		glm::vec4(  31.0f / 255.0f, 120.0f / 255.0f, 180.0f / 255.0f, 1 ),
		glm::vec4( 251.0f / 255.0f, 154.0f / 255.0f, 153.0f / 255.0f, 1 ),
		glm::vec4( 227.0f / 255.0f,  26.0f / 255.0f,  28.0f / 255.0f, 1 ),
		glm::vec4( 253.0f / 255.0f, 191.0f / 255.0f, 111.0f / 255.0f, 1 ),
		glm::vec4( 255.0f / 255.0f, 127.0f / 255.0f,   0.0f / 255.0f, 1 ),
		glm::vec4( 202.0f / 255.0f, 178.0f / 255.0f, 214.0f / 255.0f, 1 ),
		glm::vec4( 106.0f / 255.0f,  61.0f / 255.0f, 154.0f / 255.0f, 1 ),
		glm::vec4( 255.0f / 255.0f, 255.0f / 255.0f, 153.0f / 255.0f, 1 ),
		glm::vec4( 177.0f / 255.0f,  89.0f / 255.0f,  40.0f / 255.0f, 1 )
	};

	if (segment < 0)
	{
		return glm::vec4(0, 0.686f, 0, 1);
	}
	else
	{
		return colors[segment % 10];		
	}
	/* curvature visualization
	r = g = b = 100;
	if (data.caveSizeCurvatures[i] > 0)
	{
	r += (int)(data.caveSizeCurvatures[i] * 155 / 0.2);
	if (r > 255)
	r = 255;
	}*/
	/* size visualization
	r = g = b = (int)(data.caveSizes[i] * 255 / 30); */
	/* entrance prob visualization
	r = g = b = 0;
	double prob = entranceProbability(data.caveSizeCurvatures[i]);
	if (prob > 0.5)
	r = 128 + 255 * (prob - 0.5);
	else
	b = 128 + 255 * prob;*/
}

void CaveGLData::segmentationChangedHandler()
{
	std::vector<glm::vec4> meshSegColors(_meshVertices.size());
	for (int i = 0; i < _meshVertices.size(); ++i)
	{
		int segment = segmentation.at(meshVertexCorrespondsTo.at(i));
		meshSegColors.at(i) = segmentColor(segment);
	}
	meshSegColor.bind();
	meshSegColor.write(0, &meshSegColors[0], sizeof(glm::vec4) * meshSegColors.size());

	std::vector<glm::vec4> skelSegColors(skeleton->vertices.size());
	for (int i = 0; i < skeleton->vertices.size(); ++i)
	{
		int segment = segmentation.at(i);
		skelSegColors.at(i) = segmentColor(segment);
	}
	skeletonSegColor.bind();
	skeletonSegColor.write(0, &skelSegColors[0], sizeof(glm::vec4) * skelSegColors.size());
}


void CaveGLData::LoadMesh(const std::string & offFile)
{
	CaveData::LoadMesh(offFile);

	SetSkeleton(nullptr);
	segmentation.resize(0);

	ctx->MakeOpenGLContextCurrent();

	meshVB.create();
	meshVB.bind();
	std::vector<glm::vec4> positions(_meshVertices.size());
	for(int i = 0; i < _meshVertices.size(); ++i)
		positions[i] = glm::vec4(_meshVertices.at(i).x(), _meshVertices.at(i).y(), _meshVertices.at(i).z(), 1);

	meshVB.allocate(&positions[0], sizeof(glm::vec4) * positions.size());
	
	meshSegColor.create();
	meshSegColor.bind();
	meshSegColor.allocate(sizeof(glm::vec4) * positions.size());	
	
	meshIB.create();
	meshIB.bind();
	int* indices = reinterpret_cast<int*>(&_meshTriIndices[0]);
	int indexCount = sizeof(IndexedTriangle) * _meshTriIndices.size();
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

	reset_bounding_box();
	for (auto& v : _meshVertices)
		add_point(v.x(), v.y(), v.z());

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

	glDrawElements(GL_TRIANGLES, _meshTriIndices.size() * 3, GL_UNSIGNED_INT, 0);

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
	CaveData::SetSkeleton(skeleton);
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

			uint32_t* correspondingSkeletonVertices = new uint32_t[_meshVertices.size()];
			for (int v = 0; v < skeleton->vertices.size(); ++v)
				for (int corr : skeleton->vertices.at(v).correspondingOriginalVertices)
					correspondingSkeletonVertices[corr] = v;
			correspondenceVB.create();
			correspondenceVB.bind();
			correspondenceVB.allocate(correspondingSkeletonVertices, _meshVertices.size() * sizeof(uint32_t));			

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
				glVertexAttribIPointer(correspondenceProgram->attributeLocation("in_correspondence"), 1, GL_UNSIGNED_INT, 0, 0);

				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, skeletonVB.bufferId());

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
}

void CaveGLData::SmoothAndDeriveDistances()
{	
	CaveData::SmoothAndDeriveDistances();
	emit distancesChanged();
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
	glDrawElements(GL_LINES, skeleton->edges.size() * 2, GL_UNSIGNED_INT, 0);

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

	glDrawArrays(GL_POINTS, 0, skeleton->vertices.size());

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
	glDrawArrays(GL_POINTS, 0, _meshVertices.size());

	correspondenceProgram->release();
	data.correspondenceVAO.release();
}

void CaveGLData::initGL(OpenGLContextProvider* ctx, bool primary)
{	
	if (primary)
	{
		this->ctx = ctx;
		initializeOpenGLFunctions();
	}

	if (meshProgram == nullptr)
		init_shaders();

	contextSpecificData[QOpenGLContext::currentContext()].provider = ctx;
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