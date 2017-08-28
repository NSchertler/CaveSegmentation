#pragma once

#include "FileInputOutput.h"

#include <fstream>

void WriteOff(const std::string& filenameOut, const std::vector<Eigen::Vector3f>& vertices, const std::vector<IndexedTriangle>& triangles, std::function<void(int i, int& r, int& g, int& b)> colorFunc)
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

void ReadSegmentation(const std::string & filename, std::vector<int32_t>& segmentation, size_t vertexCount)
{
	segmentation.resize(vertexCount);
	FILE* file = fopen(filename.c_str(), "rb");
	fread(&segmentation[0], sizeof(int32_t), vertexCount, file);
	fclose(file);
}

void WriteSegmentation(const std::string & filename, std::vector<int32_t>& segmentation)
{
	FILE* file = fopen(filename.c_str(), "wb");
	fwrite(&segmentation[0], sizeof(int32_t), segmentation.size(), file);
	fclose(file);
}