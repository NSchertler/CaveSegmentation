#pragma once
#include "SurfaceMeshHelper.h"

#ifdef WIN32
#define NAN std::numeric_limits<Scalar>::signaling_NaN()
namespace std{  bool isnan(double x){ return _isnan(x); }
                bool isinf(double x){ return _finite(x); } }
#endif

class TopologyJanitor : public virtual SurfaceMeshHelper {
public:
    TopologyJanitor(SurfaceMeshModel* mesh) : SurfaceMeshHelper(mesh){}
	void cleanup(Scalar short_edge, Scalar edgelength_TH, Scalar alpha, const std::vector<Point>& originalPoints){
        Size nv_prev = mesh->n_vertices();
        Counter numCollapses = iteratively_coolapseShortEdges(edgelength_TH, originalPoints);
        Counter numSplits = iteratively_splitFlatTriangles(short_edge,alpha, originalPoints);
    }
protected:
    virtual ScalarHalfedgeProperty cacheAngles(Scalar short_edge){
        /// Store halfedge opposite angles
        ScalarHalfedgeProperty halpha = mesh->halfedge_property<Scalar>("h:alpha",0);
        for(Face f : mesh->faces()){
            Halfedge h_a = mesh->halfedge(f);
            Halfedge h_b = mesh->next_halfedge(h_a);
            Halfedge h_c = mesh->next_halfedge(h_b);
            Edge e_a = mesh->edge(h_a);
            Edge e_b = mesh->edge(h_b);
            Edge e_c = mesh->edge(h_c);
    
            /// Edge lengths
            Scalar a = mesh->edge_length(e_a), a2=a*a;
            Scalar b = mesh->edge_length(e_b), b2=b*b;
            Scalar c = mesh->edge_length(e_c), c2=c*c;
                
            /// A degenerate triangle will never undergo a split (but rather a collapse...)        
            if( a<short_edge || b<short_edge || c<short_edge ){
                halpha[h_a] = -1;
                halpha[h_b] = -1;
                halpha[h_c] = -1;
            } else {
                /// Opposite angles (from law of cosines)
                halpha[h_a] = acos( std::max(-1.0, std::min(1.0, (-a2 +b2 +c2)/(2*  b*c))));
                halpha[h_b] = acos( std::max(-1.0, std::min(1.0, (+a2 -b2 +c2)/(2*a  *c))));
                halpha[h_c] = acos( std::max(-1.0, std::min(1.0, (+a2 +b2 -c2)/(2*a*b  ))));
            }                
        }
        
        return halpha;
    }

	virtual Counter collapser(Scalar short_edge, const std::vector<Point>& originalPoints) = 0;
	virtual Counter splitter(Scalar short_edge, Scalar TH_ALPHA /*110*/, const std::vector<Point>& originalPoints) = 0;

protected:    
	Counter iteratively_splitFlatTriangles(Scalar short_edge /*1e-10*/, Scalar TH_ALPHA /*110*/, const std::vector<Point>& originalPoints){
        Counter new_splits=0;
        Counter tot_splits=0;
        do{
            new_splits = splitter(short_edge,TH_ALPHA, originalPoints);
            tot_splits += new_splits;
            // qDebug() << "new splits: " << new_collapses;
        } while(new_splits>0);
        return tot_splits;
    }
	Counter iteratively_coolapseShortEdges(Scalar edgelength_TH, const std::vector<Point>& originalPoints){
        /// This could be done more efficiently, by keeping 
        /// track of possible rings where new collapses took
        /// place... but I am lazy...
        Counter new_collapses=0;
        Counter tot_collapses=0;
        do{
            new_collapses = collapser(edgelength_TH, originalPoints);
            tot_collapses += new_collapses;
            // qDebug() << "new collapses: " << new_collapses;
        } while(new_collapses>0);
        return tot_collapses;
    }
};
