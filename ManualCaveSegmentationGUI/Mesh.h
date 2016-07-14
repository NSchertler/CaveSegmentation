#pragma once

#include <IHasBoundingBox.h>

#include <string>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

#include <CameraProvider.h>
#include <GLView.h>

#include "kdtree.h"


class Mesh : public QObject, public IHasBoundingBox, public QOpenGLFunctions_3_3_Core
{
	Q_OBJECT

public:
	Mesh(GLView* parent, std::string filename);
	~Mesh();
	static void init_shaders(QOpenGLContext* context);
	
	void createBuffers();

	void draw(CameraProvider* cam, bool transparent);
	void Mesh::paint(int mouseX, int mouseY, int radius, int value);	

	void saveSegmentation(std::string filename);
	void loadSegmentation(std::string filename);

	size_t getSegmentationData(const unsigned int*&) const;
	void getAxisEnds(CameraProvider* cam, int& northX, int& northY, int& eastX, int& eastY);

private:
	QOpenGLBuffer indexBuffer;
	QOpenGLBuffer positionBuffer;
	QOpenGLBuffer segmentationBuffer;
	QOpenGLVertexArrayObject vao;
	
	std::vector<glm::vec3> positions;
	std::vector<unsigned int> indices;
	std::vector<float> screenSpaceDepths;
	std::vector<unsigned int> segmentation;

	static std::unique_ptr<QOpenGLShaderProgram> program;

	unsigned int n_indices;	

	GLView* parent;	

	kdtree* screenSpaceTree;

	bool openGLReady;

	void readFile(std::string filename);

private slots:
	void parent_viewUpdated();
};