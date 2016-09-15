#pragma once
#include "SurfaceMeshHelper.h"

using namespace SurfaceMesh;

class  CotangentLaplacianHelper : public SurfaceMeshHelper{    
public:
    CotangentLaplacianHelper(SurfaceMeshModel* mesh) : SurfaceMeshHelper(mesh){}
        
    ScalarHalfedgeProperty computeCotangentEdgeWeights(const std::string property="e:weight"){
        ScalarHalfedgeProperty hweight = mesh->add_halfedge_property<Scalar>(property);
        for(Edge e : mesh->edges()){
            hweight[ mesh->halfedge(e,0) ] = cotangentLaplacianWeight(e);
            hweight[ mesh->halfedge(e,1) ] = cotangentLaplacianWeight(e);
        }
        return hweight;
    }
    
    Vector3VertexProperty computeLaplacianVectors(ScalarHalfedgeProperty hweight, const std::string property="v:laplace", bool autonormalize=true){
        Vector3VertexProperty laplace = mesh->vertex_property<Vector3>(property);
        for(Vertex v : mesh->vertices())
            laplace[v] = cotangentLaplacianVector(hweight,v,autonormalize);
        return laplace;
    }
    
protected:
    Scalar cotangentLaplacianWeight(Edge eit){
        ElementProperty<Point, Vertex> points = mesh->vertex_property<Point>("v:point");
       
        bool secure = true;
        
        Scalar lb(-0.999), ub(0.999);        
        Scalar w  = 0.0;
        Surface_mesh::Halfedge h0 = mesh->halfedge(eit, 0);
        Surface_mesh::Vertex   v0 = mesh->to_vertex(h0);
        Point p0 = points[v0];
        
        Surface_mesh::Halfedge h1 = mesh->halfedge(eit, 1);
        Surface_mesh::Vertex   v1 = mesh->to_vertex(h1);
        Point p1 = points[v1];
        
        if(mesh->is_boundary(h0) || mesh->is_boundary(h1))
            throw "Not supported here!!! (mesh with boundary? try to clean with MeshFix, an external program)";
            
        if (!mesh->is_boundary(h0))
        {
            Point p2 = points[mesh->to_vertex(mesh->next_halfedge(h0))];
            Point d0 = (p0 - p2).normalized();
            Point d1 = (p1 - p2).normalized();
            Scalar c  = dot(d0,d1);
			if (!std::isnan(c))
			{
				if (secure) {
					c = (c < lb) ? lb : c;
					c = (c > ub) ? ub : c;
				}
				w += 1.0 / tan(acos(c));
			}
        }
        
        if (!mesh->is_boundary(h1))
        {
            Point p2 = points[mesh->to_vertex(mesh->next_halfedge(h1))];
            Point d0 = (p0 - p2).normalized();
            Point d1 = (p1 - p2).normalized();
            Scalar c  = dot(d0,d1);
			if (!std::isnan(c))
			{
				if (secure) {
					c = (c < lb) ? lb : c;
					c = (c > ub) ? ub : c;
				}
				w += 1.0 / tan(acos(c));
			}
        }
        
        // force weights to be non-negative for higher robustness
        if(secure) if (w < 0.0) w = 0.0;
        return w;
    }
    
    Vector3 cotangentLaplacianVector(ScalarHalfedgeProperty hweight, Vertex vit, bool autoNormalize){
        Point l(0,0,0);
        if( autoNormalize ){
            Scalar w=0;
            if(!mesh->is_boundary(vit)){
                for(Halfedge hvit : mesh->onering_hedges(vit)){
                    Vertex v = mesh->to_vertex(hvit);
                    l += hweight[hvit] * (points[v] - points[vit]);
                    w += hweight[hvit];
                }                
                l /= w;
            }
        } else {
            if(!mesh->is_boundary(vit)){
                for(Halfedge hvit : mesh->onering_hedges(vit)){
                    Vertex v = mesh->to_vertex(hvit);
                    l += hweight[hvit] * (points[v] - points[vit]) / (2*varea[vit]);
                }                
            }
        }
        return l;
    }      
};
