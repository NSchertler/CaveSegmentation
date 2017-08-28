#pragma once

#include <CGAL/Simple_cartesian.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Triangle_3 Triangle;
typedef K::Vector_2 Vector2;

typedef std::vector<Triangle> TriangleList;