#include "Mesh.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <QMessageBox>
#include <queue>
#include <glm/gtc/type_ptr.hpp>
#include <GLView.h>
#include <GLUtils.h>
#include <iostream>


std::unique_ptr<QOpenGLShaderProgram> Mesh::program(nullptr);

Mesh::Mesh(GLView* parent, std::string filename)
	: parent(parent), screenSpaceTree(kd_create(2)), indexBuffer(QOpenGLBuffer::IndexBuffer), positionBuffer(QOpenGLBuffer::VertexBuffer), segmentationBuffer(QOpenGLBuffer::VertexBuffer), openGLReady(false)
{
	connect(parent, &GLView::CameraStill, this, &Mesh::Parent_CameraStill);

	readFile(filename);
}

Mesh::~Mesh()
{
	kd_free(screenSpaceTree);	
}

void Mesh::Parent_CameraStill()
{	
	//Build screen space kd tree
	kd_clear(screenSpaceTree);

	glm::mat4 MVP = parent->GetProjectionMatrix() * parent->GetViewMatrix();

	for (int i = 0; i < positions.size(); ++i)
	{
		auto posInViewSpace = parent->GetViewMatrix() * glm::vec4(positions[i], 1.0f);
		auto posInClipSpace = parent->GetProjectionMatrix() * posInViewSpace;
		posInClipSpace *= 1.0f / posInClipSpace.w;

		posInClipSpace.x = round(parent->width() * (0.5f * (posInClipSpace.x + 1.0f)));
		posInClipSpace.y = round(parent->height() * (0.5f * (posInClipSpace.y + 1.0f)));
		posInClipSpace.z = 0.5f * (posInClipSpace.z + 1.0f);

		if (posInClipSpace.x >= 0 && posInClipSpace.x < parent->width() && posInClipSpace.y >= 0 && posInClipSpace.y < parent->height())
		{
			kd_insertf(screenSpaceTree, glm::value_ptr(posInClipSpace), &positions[i]);
			screenSpaceDepths[i] = posInViewSpace.z;
		}
	}

}

void Mesh::getAxisEnds(CameraProvider* cam, int& northX, int& northY, int& eastX, int& eastY)
{
	auto MVP = cam->GetProjectionMatrix() * cam->GetViewMatrix();

	auto min = getMin();
	auto max = getMax();

	//north
	auto posOnScreen = MVP * glm::vec4(min.x, max.y, min.z, 1.0f);
	posOnScreen *= 1.0f / posOnScreen.w;
	northX = (0.5f * posOnScreen.x + 0.5f) * parent->width();
	northY = (0.5f - 0.5f * posOnScreen.y) * parent->height();

	//east
	posOnScreen = MVP * glm::vec4(max.x, min.y, min.z, 1.0f);
	posOnScreen *= 1.0f / posOnScreen.w;

	eastX = (0.5f * posOnScreen.x + 0.5f) * parent->width();
	eastY = (0.5f - 0.5f * posOnScreen.y) * parent->height();
}

size_t Mesh::getSegmentationData(const unsigned int*& data) const
{
	data = &segmentation[0];
	return segmentation.size();
}

void Mesh::saveSegmentation(std::string filename)
{
	std::ofstream f(filename, std::ios::binary);
	f.write(reinterpret_cast<char*>(&segmentation[0]), sizeof(unsigned int) * segmentation.size());
	f.close();
}

void Mesh::loadSegmentation(std::string filename)
{
	std::ifstream f(filename, std::ios::binary);
	f.read(reinterpret_cast<char*>(&segmentation[0]), sizeof(unsigned int) * segmentation.size());
	f.close();

	segmentationBuffer.bind();
	segmentationBuffer.write(0, &segmentation[0], segmentation.size() * sizeof(unsigned int));
	segmentationBuffer.release();	
}


void Mesh::paint(int mouseX, int mouseY, int radius, int value)
{
	bool changed = false;

	float mousePos[2] = { (float)mouseX, (float)mouseY };
	auto candidates = kd_nearest_rangef(screenSpaceTree, mousePos, radius);

	float zTolerance = 0.005 * (parent->getZFar() - parent->getZNear()); //The allowed tolerance between actual depth buffer and theoretical value.

	auto proj = parent->GetProjectionMatrix();

	float vertexScreenPos[2];
	while (!kd_res_end(candidates))
	{
		auto positionPointer = (glm::vec3*)kd_res_itemf(candidates, vertexScreenPos);
		int index = positionPointer - &positions[0];

		//check vertex orientation
		if (glm::dot(parent->getEye() - *positionPointer, vertexNormals.at(index)) > 0)
		{

			if (segmentation[index] != value)
			{
				float depthMin = ((screenSpaceDepths[index] + zTolerance) * proj[2][2] + proj[3][2]) / ((screenSpaceDepths[index] + zTolerance) * proj[2][3] + proj[3][3]);
				float depthMax = ((screenSpaceDepths[index] - zTolerance) * proj[2][2] + proj[3][2]) / ((screenSpaceDepths[index] - zTolerance) * proj[2][3] + proj[3][3]);
				float actualDepth = 2 * parent->depthBuffer()[(int)(vertexScreenPos[0] + vertexScreenPos[1] * parent->width())] - 1;
				if (depthMin <= actualDepth && actualDepth <= depthMax)
				{
					segmentation[index] = value;
					changed = true;
				}
			}
		}

		kd_res_next(candidates);
	}

	kd_res_free(candidates);

	if (changed)
	{
		segmentationBuffer.bind();
		segmentationBuffer.write(0, &segmentation[0], segmentation.size() * sizeof(unsigned int));
		segmentationBuffer.release();
	}

	parent->repaint();
}

void Mesh::init_shaders(QOpenGLContext* context)
{
	program = MakeProgram(":/glsl/mesh.vert", ":/glsl/mesh.frag", ":/glsl/mesh.geom");		
}

void Mesh::createBuffers()
{
	initializeOpenGLFunctions();

	vao.create();
	vao.bind();

	indexBuffer.create();
	indexBuffer.bind();
	indexBuffer.allocate(&indices[0], indices.size() * sizeof(unsigned int));

	positionBuffer.create();
	positionBuffer.bind();
	positionBuffer.allocate(&positions[0], positions.size() * sizeof(glm::vec3));
	program->enableAttributeArray("in_position");
	program->setAttributeBuffer("in_position", GL_FLOAT, 0, 3);

	segmentationBuffer.create();
	segmentationBuffer.bind();
	segmentationBuffer.allocate(segmentation.size() * sizeof(unsigned int));
	program->enableAttributeArray("segmentation");
	glVertexAttribIPointer(program->attributeLocation("segmentation"), 1, GL_UNSIGNED_INT, 0, 0);

	vao.release();

	Parent_CameraStill();

	openGLReady = true;
}

void readOff(std::string filename, std::vector<glm::vec3>& vertices, std::vector<unsigned int>& indices)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	if (!f.good())
		throw;
	std::string line;
	int nVertices = -1, nFaces = -1, nEdges = -1;

	std::vector<glm::vec3>::iterator nextVertex;
	auto nextIndex = indices.begin();

	while (std::getline(f, line))
	{
		if (line.size() == 0)
			continue;
		if (line[0] == '#')
			continue;
		if (line == "OFF")
			continue;

		std::stringstream str(line);
		if (nVertices < 0)
		{
			str >> nVertices >> nFaces >> nEdges;
			vertices.resize(nVertices);
			indices.resize(3 * nFaces);
			nextVertex = vertices.begin();
			nextIndex = indices.begin();
		}
		else
		{
			if (nVertices > 0)
			{
				str >> nextVertex->x >> nextVertex->y >> nextVertex->z;
				++nextVertex;
				--nVertices;
			}
			else if (nFaces > 0)
			{
				int n, a, b, c;
				str >> n >> a >> b >> c;
				if (n != 3)
					throw;
				(*nextIndex++) = a;
				(*nextIndex++) = b;
				(*nextIndex++) = c;
				
				--nFaces;
			}
		}
	}
	f.close();
}

void read_obj(std::string filename, std::vector<glm::vec3>& positions, std::vector<unsigned int>& indices)
{
	std::ifstream f(filename);

	std::string line;
	
	glm::vec3 p;
	unsigned int i, j, k;

	while (std::getline(f, line))
	{		
		if (line.length() == 0 || line[0] == '#')
			continue;
		else
		{
			std::stringstream ss(line);
			ss.ignore(1);
			if (line[0] == 'v')
			{
				ss >> p.x >> p.y >> p.z;
				positions.push_back(p);				
			}
			else if (line[0] == 'f')
			{
				ss >> i >> j >> k;
				indices.push_back(i - 1);
				indices.push_back(j - 1);
				indices.push_back(k - 1);
			}
		}
	}

	f.close();
}


void readBin(std::string filename, std::vector<glm::vec3>& positions, std::vector<unsigned int>& indices)
{
	std::ifstream f(filename, std::ios::binary);

	int nVertices, nIndices;
	f.read(reinterpret_cast<char*>(&nVertices), sizeof(int));
	f.read(reinterpret_cast<char*>(&nIndices), sizeof(int));

	positions.resize(nVertices);
	indices.resize(nIndices);

	f.read(reinterpret_cast<char*>(&positions[0]), sizeof(glm::vec3) * nVertices);
	f.read(reinterpret_cast<char*>(&indices[0]), sizeof(unsigned int) * nIndices);

	f.close();
}


bool hasEnding(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
	}
	else {
		return false;
	}
}

void Mesh::readFile(std::string filename)
{
	positions.clear();	
	
	if (hasEnding(filename, ".obj"))
		read_obj(filename, positions, indices);
	else if (hasEnding(filename, ".off"))
		readOff(filename, positions, indices);
	else if (hasEnding(filename, ".bin"))
		readBin(filename, positions, indices);
	else
		throw;

	reset_bounding_box();
	for (auto& v : positions)
		add_point(v);


#ifdef ADMIN
	{
		std::ofstream binaryOutput("cave.bin", std::ios::binary);
		
		int nVertices = positions.size();
		int nIndices = indices.size();
		binaryOutput.write(reinterpret_cast<char*>(&nVertices), sizeof(int));
		binaryOutput.write(reinterpret_cast<char*>(&nIndices), sizeof(int));
		binaryOutput.write(reinterpret_cast<char*>(&positions[0]), nVertices * sizeof(glm::vec3));
		binaryOutput.write(reinterpret_cast<char*>(&indices[0]), nIndices * sizeof(unsigned int));

		binaryOutput.close();
	}
#endif

	n_indices = indices.size();
	segmentation.clear();
	segmentation.resize(positions.size(), 0);
	screenSpaceDepths.resize(positions.size());

	vertexNormals.clear();
	vertexNormals.resize(positions.size(), glm::vec3(0, 0, 0));

	// Calculate vertex normals
	for (int i = 0; i < n_indices; i += 3)
	{
		glm::vec3 n = glm::cross(positions[indices[i + 1]] - positions[indices[i]], positions[indices[i + 2]] - positions[indices[i]]);
		vertexNormals.at(indices[i]) += n;
		vertexNormals.at(indices[i + 1]) += n;
		vertexNormals.at(indices[i + 2]) += n;
	}
	for (int i = 0; i < positions.size(); ++i)
		vertexNormals.at(i) = glm::normalize(vertexNormals.at(i));
}

void Mesh::draw(CameraProvider* cam, bool transparent)
{
	if (openGLReady)
	{
		program->bind();

		auto MVP = glm::transpose(cam->GetProjectionMatrix() * cam->GetViewMatrix());
		auto m = QMatrix4x4(glm::value_ptr(MVP));
		program->setUniformValue("mvp", m);
		program->setUniformValue("transparent", transparent);

		vao.bind();

		glDrawElements(GL_TRIANGLES, n_indices, GL_UNSIGNED_INT, 0);

		vao.release();
		program->release();

		/*auto min = this->getMin();
		auto max = this->getMax();
		glBegin(GL_LINES);

		glColor3f(1.0f, 0.5f, 0.3f);
		glVertex3f(min.x, min.y, min.z);
		glVertex3f(max.x, min.y, min.z);

		glColor3f(1.0f, 0.5f, 0.3f);
		glVertex3f(min.x, min.y, min.z);
		glVertex3f(min.x, max.y, min.z);

		glColor3f(1.0f, 0.5f, 0.3f);
		glVertex3f(min.x, min.y, min.z);
		glVertex3f(min.x, min.y, max.z);

		glEnd();*/
	}
}