#pragma once
#include <set>
#include "SurfaceMeshHelper.h"

namespace SurfaceMesh
{
	template <class Mesh>
	class EdgePropertyPriorityQueue
	{
	public:
		class EdgeCompareFunctor
		{
		public:
			ElementProperty<typename Mesh::Scalar, typename Mesh::Edge> scalars;

			EdgeCompareFunctor(Mesh* mesh, std::string property){
				scalars = mesh->edge_property<Mesh::Scalar>(property);
			}

			bool operator() (const typename Mesh::Edge& lhs, const typename Mesh::Edge& rhs) const{
				Mesh::Scalar p0 = scalars[lhs];
				Mesh::Scalar p1 = scalars[rhs];
				return (p0 == p1) ? (lhs.idx() < rhs.idx()) : (p0 < p1);
			}
		};

		EdgeCompareFunctor compareFunctor;
		std::set<typename Mesh::Edge, EdgeCompareFunctor> set;

		EdgePropertyPriorityQueue(Mesh* mesh, std::string property)
			: compareFunctor(mesh, property), set(compareFunctor){}

		void insert(typename Mesh::Edge edge, typename Mesh::Scalar scalar){
			compareFunctor.scalars[edge] = scalar;
			set.insert(edge);
		}

		bool update(typename Mesh::Edge edge, typename Mesh::Scalar newScalar){
			// erase the edge
			int nerased = set.erase(edge);

			if (nerased != 1){
				return false;
			}

			compareFunctor.scalars[edge] = newScalar;

			// re-insert it
			set.insert(edge);

			return true;
		}

		bool empty(){
			return set.empty();
		}

		typename Mesh::Edge pop(){
			typename Mesh::Edge e = *(set.begin());
			set.erase(e);
			compareFunctor.scalars[e] = -1.0;
			return e;
		}

		bool has(typename Mesh::Edge edge)
		{
			return set.find(edge) != set.end();
		}
	};
}
