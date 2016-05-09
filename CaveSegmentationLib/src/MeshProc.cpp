#include "MeshProc.h"

double GetSqrDistanceToMesh(const Point& p, const Vector& dir, const Tree& tree)
{
	typedef K::Ray_3 Ray;
	typedef Tree::Intersection_and_primitive_id<Ray>::Type IntersectionType;

	class RememberMinDistanceIterator
		: public std::iterator<std::output_iterator_tag, IntersectionType>
	{
	public:
		RememberMinDistanceIterator(const Point& origin, double& min)
			: origin(origin), min(min)
		{}

		RememberMinDistanceIterator& operator=(const IntersectionType& o)
		{
			Point p;
			CGAL::assign(p, o.first);

			auto length = (p - origin).squared_length();
			if (length < min)
				min = length;

			return *this;
		}

		RememberMinDistanceIterator& operator*() { return *this; }
		RememberMinDistanceIterator& operator++() { return *this; }
		RememberMinDistanceIterator& operator++(int) { return *this; }

		double minDistance() const { return min; }
	protected:
		const Point& origin;
		double& min;
	};

	double minDistance = std::numeric_limits<double>::infinity();
	RememberMinDistanceIterator min(p, minDistance);

	Ray ray_query(p, dir);

	tree.all_intersections(ray_query, min);

	return minDistance;
}