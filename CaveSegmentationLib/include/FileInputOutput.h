#pragma once

#include "CGALCommon.h"
#include <CurveSkeleton.h>
#include <Eigen/Dense>
#include <fstream>

void ReadOff(std::string filename, std::vector<Eigen::Vector3f>& vertices, std::vector<Triangle>& triangles, std::vector<IndexedTriangle>& triIndices)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	if (!f.good())
		throw;
	std::string line;
	int nVertices = -1, nFaces = -1, nEdges = -1;

	std::vector<Eigen::Vector3f>::iterator nextVertex;
	auto nextTriangle = triangles.begin();
	auto nextTriIndex = triIndices.begin();

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
			triangles.resize(nFaces);
			triIndices.resize(nFaces);
			nextVertex = vertices.begin();
			nextTriangle = triangles.begin();
			nextTriIndex = triIndices.begin();
		}
		else
		{
			if (nVertices > 0)
			{
				str >> nextVertex->x() >> nextVertex->y() >> nextVertex->z();
				++nextVertex;
				--nVertices;
			}
			else if (nFaces > 0)
			{
				int n, a, b, c;
				str >> n >> a >> b >> c;
				if (n != 3)
					throw;
				(*nextTriangle) = Triangle(
					Point(vertices[a].x(), vertices[a].y(), vertices[a].z()),
					Point(vertices[b].x(), vertices[b].y(), vertices[b].z()),
					Point(vertices[c].x(), vertices[c].y(), vertices[c].z()));
				(*nextTriIndex).i[0] = a;
				(*nextTriIndex).i[1] = b;
				(*nextTriIndex).i[2] = c;

				++nextTriangle;
				++nextTriIndex;
				--nFaces;
			}
		}
	}
	f.close();
}

void WriteOff(std::string filenameOut, std::vector<Eigen::Vector3f>& vertices, std::vector<IndexedTriangle>& triangles, std::function<void(int i, int& r, int& g, int& b)> colorFunc)
{
	std::ofstream fOut;
	fOut.open(filenameOut, std::ios::out);
	if (!fOut.good())
	{
		throw;
	}
	std::string line;

	fOut << "COFF" << std::endl;
	fOut << vertices.size() << " " << triangles.size() << " 0" << std::endl;

	int iVert = 0;
	for (auto& v : vertices)
	{
		int r, g, b;
		colorFunc(iVert++, r, g, b);
		fOut << v.x() << " " << v.y() << " " << v.z() << " " << r << " " << g << " " << b << std::endl;
	}
	for (auto& f : triangles)
	{
		fOut << "3 " << f.i[0] << " " << f.i[1] << " " << f.i[2] << std::endl;
	}

	fOut.close();
}

void ReadSegmentation(const std::string & filename, std::vector<int32_t>& segmentation, CurveSkeleton * skeleton)
{
	segmentation.resize(skeleton->vertices.size());
	FILE* file = fopen(filename.c_str(), "rb");
	fread(&segmentation[0], sizeof(int32_t), skeleton->vertices.size(), file);
	fclose(file);
}

void WriteSegmentation(const std::string & filename, std::vector<int32_t>& segmentation)
{
	FILE* file = fopen(filename.c_str(), "wb");
	fwrite(&segmentation[0], sizeof(int32_t), segmentation.size(), file);
	fclose(file);
}