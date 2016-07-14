#pragma once

#include <QObject>
#include <memory>
#include "Mesh.h"

class GLCaveView;

class SharedData : public QObject
{
private:
	Q_OBJECT

		std::shared_ptr<Mesh> mesh;

public:
	GLCaveView* view;
	int currentCaveId;

	SharedData()
		: mesh(std::shared_ptr<Mesh>(nullptr)), currentCaveId(-1)
	{

	}

	void setMesh(std::shared_ptr<Mesh> mesh) { this->mesh = mesh; emit meshChanged(); }
	std::shared_ptr<Mesh> getMesh() const { return mesh; }

signals:
	void meshChanged();
};