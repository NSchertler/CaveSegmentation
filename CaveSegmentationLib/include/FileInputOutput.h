#pragma once

#include "IndexedTriangle.h"
#include <Eigen/Dense>
#include <CurveSkeleton.h>

#include "CaveSegmentationLib.h"

extern CAVESEGMENTATIONLIB_API void WriteOff(const std::string& filenameOut, const std::vector<Eigen::Vector3f>& vertices, const std::vector<IndexedTriangle>& triangles, std::function<void(int i, int& r, int& g, int& b)> colorFunc);

extern CAVESEGMENTATIONLIB_API void ReadSegmentation(const std::string & filename, std::vector<int32_t>& segmentation, size_t vertexCount);

extern CAVESEGMENTATIONLIB_API void WriteSegmentation(const std::string & filename, std::vector<int32_t>& segmentation);