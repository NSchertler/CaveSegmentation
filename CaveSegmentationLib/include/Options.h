#pragma once

#ifndef _USE_MATH_DEFINES
#error "Please define _USE_MATH_DEFINES in the project settings."
#endif

#include <cmath>

#define NON_VERBOSE

//This option causes the program to write debug images during processing
//#define DRAW_DEBUG_IMAGES

//This option causes the program to write statistics during LineFlow on a sphere to a CSV file
//#define WRITE_SPHERE_STATS

//This option causes the program to write a 3D model of the surrounding spheres of skeleton vertices.
//#define WRITE_SPHERE_VIS

//This option causes the program to write a 3D model of the surrounding spheres of skeleton vertices (mapped to a planar height field).
//#define WRITE_HEIGHTFIELD

#ifdef DRAW_DEBUG_IMAGES
const int imWidth = 1280;
const int imHeight = imWidth / 2;
#endif

//The resolution of the regular sphere sampling used for cave size calculation
const int SPHERE_SAMPLING_RESOLUTION = 51;

//Minimum angular distance of samples when finding representatives on the ridge line
const double CIRCLE_SUBSAMPLING_MIN_DISTANCE = M_PI / 3;
//Maximum angular distance of samples when finding representatives on the ridge line
const double CIRCLE_SUBSAMPLING_MAX_DISTANCE = 4 * M_PI / 3;