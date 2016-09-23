#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#define NON_VERBOSE

//#define WRITE_BRANCH_STATISTICS
//#define DRAW_DEBUG_IMAGES
//#define WRITE_SPHERE_STATS
//#define WRITE_SPHERE_VIS
//#define WRITE_HEIGHTFIELD
//#define EXPORT_BRANCHES
//#define CALC_SKELETON
//#define CALC_DISTANCES

#ifdef DRAW_DEBUG_IMAGES
const int imWidth = 1280;
const int imHeight = imWidth / 2;
#endif

const int SPHERE_SAMPLING_RESOLUTION = 51;

const double CIRCLE_SUBSAMPLING_MIN_DISTANCE = M_PI / 3;
const double CIRCLE_SUBSAMPLING_MAX_DISTANCE = 4 * M_PI / 3;