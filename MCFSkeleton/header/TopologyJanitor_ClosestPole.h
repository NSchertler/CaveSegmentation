#pragma once
#include "TopologyJanitor.h"
#include "Skelcollapse.h"
#include <map>
class TopologyJanitor_ClosestPole : public TopologyJanitor{
public:
    TopologyJanitor_ClosestPole(SurfaceMeshModel* mesh) : SurfaceMeshHelper(mesh), TopologyJanitor(mesh){}
       
    /// @{ This collapse mode retains only the closest pole greedily
	virtual Counter collapser(Scalar edgelength_TH, const std::vector<Point>& originalPoints){
        Vector3VertexProperty poles = mesh->get_vertex_property<Vector3>("v:pole");

        /// Retrieve memory to store correspondences (must be already initialized)
        VertexListVertexProperty corrs = mesh->vertex_property<VertexList>("v:corrs");
        if(!corrs) throw MissingPropertyException("v:corrs");
        
        Counter count=0;
        for(Edge e : mesh->edges()){
            Halfedge h = mesh->halfedge(e,0);
            if(mesh->edge_length(e)<edgelength_TH){
				count += collapseEdge(mesh, h, poles, corrs) ? 1 : 0;
            }
        }
        return count;
    }

	bool collapseEdge(SurfaceMeshModel* mesh, Halfedge h, Vector3VertexProperty& poles, VertexListVertexProperty& corrs)
	{
		if (mesh->is_deleted(h) || !mesh->is_collapse_ok(h))
			return false;

		Vertex v0 = mesh->from_vertex(h);
		Vertex v1 = mesh->to_vertex(h);

		std::map<Vertex, double> distances;
		for (auto n : mesh->onering_hedges(v0))
		{
			double l = mesh->edge_length(mesh->edge(n));
			distances[mesh->to_vertex(n)] = l;
		}
		for (auto n : mesh->onering_hedges(v1))
		{
			double l = mesh->edge_length(mesh->edge(n));
			Vertex v = mesh->to_vertex(n);
			if (distances.find(v) == distances.end() || distances[v] < l)
				distances[v] = l;
		}

		points[v1] = (points[v0] + points[v1]) / 2.0f;

		/// Find the closest pole
		Vector3 pole0 = poles[v0];
		Vector3 pole1 = poles[v1];
		Scalar d0 = (pole0 - points[v1]).norm();
		Scalar d1 = (pole1 - points[v1]).norm();

		/// Pick it
		poles[v1] = (d0<d1) ? poles[v0] : poles[v1];

		/// And keep track of correspondences
		corrs[v1].insert(corrs[v1].end(), corrs[v0].begin(), corrs[v0].end());

		/// Perform collapse
		mesh->collapse(h);

		auto start = mesh->halfedges(v1);
		auto it = start;

		/*for (auto n : mesh->onering_hedges(v1))
		{
		double l = mesh->edge_length(mesh->edge(n));
		if (l > 5)
		{
		double oldL = distances[mesh->to_vertex(n)];
		std::cout << "New neighbor edge length: ";
		std::cout << oldL << " --> " << l << " (changed by " << (l - oldL) << "; threshold: " << edgelength_TH << ")" << std::endl;
		}
		}*/
		return true;
	}

	void updateCorrespondences(VertexList& source, VertexList& target, const std::vector<Point>& originalPoints, const Point& newpos, const Point& p0)
	{
		auto it = source.begin();
		while (it != source.end())
		{
			auto p = originalPoints[*it];
			if ((p - newpos).norm() < (p - p0).norm())
			{
				target.push_back(*it);
				it = source.erase(it);
			}
			else
				++it;
		}
	}

	virtual Counter splitter(Scalar short_edge, Scalar TH_ALPHA /*110*/, const std::vector<Point>& originalPoints){
        Vector3VertexProperty poles  = mesh->get_vertex_property<Vector3>("v:pole");

        /// Keep track / decide which to split    
        TH_ALPHA *= (3.14/180);
        
        /// Store halfedge opposite angles
        ScalarHalfedgeProperty halpha = cacheAngles(short_edge);

        /// Splitting section
        Scalar numsplits=0;
        BoolVertexProperty vissplit = mesh->vertex_property<bool>("v:issplit",false);
		auto corrs = mesh->vertex_property<VertexList>("v:corrs");
        for(Edge e : mesh->edges()){
            Halfedge h0 = mesh->halfedge(e,0);
            Halfedge h1 = mesh->halfedge(e,1);
    
            /// Should a split take place?
            Scalar alpha_0 = halpha[ h0 ];
            Scalar alpha_1 = halpha[ h1 ];
			
			//Original: 
			if (alpha_0<TH_ALPHA || alpha_1<TH_ALPHA) continue;
            //if((alpha_0<TH_ALPHA && alpha_1<TH_ALPHA) || alpha_0 < 1.0 || alpha_1 < 1.0) continue;
          
            /// Which side should I split?
            Vertex w0 = mesh->to_vertex( mesh->next_halfedge(h0) );
            Vertex w1 = mesh->to_vertex( mesh->next_halfedge(h1) );
            Vertex wsplitside = (alpha_0>alpha_1) ? w0 : w1;
            
            /// Project side vertex on edge
			Vertex v0 = mesh->vertex(e, 0);
			Vertex v1 = mesh->vertex(e, 1);
            Point p0 = points[v0];
            Point p1 = points[v1];
            Vector3 projector = (p1-p0).normalized();
            Vector3 projectee = points[wsplitside]-p0;
            Scalar t = dot(projector, projectee);           			

            assert(!std::isnan(t));
            Vector3 newpos = p0 + t*projector;

            /// Perform the split at the desired location
            Vertex vnew = mesh->split(e,newpos);
            
            /// Also project the pole
            Vector3 pole0 = poles[mesh->vertex(e,0)];
            Vector3 pole1 = poles[mesh->vertex(e,1)];            
            Vector3 p_projector = (pole1-pole0).normalized();
            poles[vnew] = pole0 + t*p_projector; 

			//Update correspondences
			updateCorrespondences(corrs[v0], corrs[vnew], originalPoints, newpos, p0);
			updateCorrespondences(corrs[v1], corrs[vnew], originalPoints, newpos, p1);

            /// And mark it as a split
            vissplit[vnew] = true;
            numsplits++;
        }
        return numsplits;
    }
};
