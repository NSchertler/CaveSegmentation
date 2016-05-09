#pragma once

#include "CGALCommon.h"
#include <Eigen/Dense>
#include <CurveSkeleton.h>

extern void ReadOff(std::string filename, std::vector<Eigen::Vector3f>& vertices, std::vector<Triangle>& triangles, std::vector<IndexedTriangle>& triIndices);

extern void WriteOff(const std::string& filenameOut, const std::vector<Eigen::Vector3f>& vertices, const std::vector<IndexedTriangle>& triangles, std::function<void(int i, int& r, int& g, int& b)> colorFunc);

extern void ReadSegmentation(const std::string & filename, std::vector<int32_t>& segmentation, CurveSkeleton * skeleton);

extern void WriteSegmentation(const std::string & filename, std::vector<int32_t>& segmentation);