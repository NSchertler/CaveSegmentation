#include "Skelcollapse.h"
#include "TopologyJanitor.h"
#include "TopologyJanitor_ClosestPole.h"
#include "QhullVoronoiHelper.h"
#include "EigenContractionHelper.h"
#include "EdgePropertyPriorityQueue.h"
#include "WingedEdgeMeshModel.h"

void Skelcollapse::calculateVoronoi(){
	// We need normals
	mesh->update_face_normals();
	mesh->update_vertex_normals();

	VoronoiHelper h(mesh);
	h.computeVoronoiDiagram();
	h.searchVoronoiPoles();
	h.getMedialSpokeAngleAndRadii();
	h.setToMedial();

	poles = helper.getVector3VertexProperty("v:pole");
}

void Skelcollapse::contractGeometry(){
	EigenContractionHelper(mesh).evolve(omega_H, omega_L, omega_P, poles);
}

void Skelcollapse::updateConstraints(){
	for(Vertex v : mesh->vertices()){
		/// Leave fixed points really alone
		if (visfixed[v]){
			omega_L[v] = 0;
			omega_H[v] = 1.0 / zero_TH;
			omega_P[v] = 0;
			continue;
		}

		omega_L[v] = omega_L_0;
		omega_H[v] = omega_H_0;
		omega_P[v] = omega_P_0;

		/// Ficticious vertices are simply relaxed
		if (vissplit[v]){
			omega_L[v] = omega_L_0;
			omega_H[v] = omega_H_0;
			omega_P[v] = 0;
		}
	}
}

void Skelcollapse::detectDegeneracies(){
	Scalar elength_fixed = edgelength_TH / 10.0;

	fixedVertices = 0;

	for(Vertex v : mesh->vertices()){
		/// previously fixed remain so
		if (visfixed[v]) 
		{
			++fixedVertices;
			continue;
		}

		bool willbefixed = false;
		Counter badcounter = 0;
		for(Halfedge h : mesh->onering_hedges(v)){
			Scalar elength = mesh->edge_length(mesh->edge(h));
			if (elength<elength_fixed && !mesh->is_collapse_ok(h))
				badcounter++;
		}
		willbefixed = (badcounter >= 2);
		visfixed[v] = willbefixed;

		if (visfixed[v])
			++fixedVertices;
	}
}
void Skelcollapse::updateTopology(const std::vector<Point>& originalPoints){
	TopologyJanitor_ClosestPole(mesh).cleanup(zero_TH, edgelength_TH, 110, originalPoints);
}

void Skelcollapse::contract()
{
	auto points = mesh->get_vertex_property<SurfaceMeshModel::Point>("v:point");
	 originalPoints.resize(mesh->n_vertices());
	
	if (!isInitialized){

		/// Every vertex initially corresponds to itself
		for (Vertex v : mesh->vertices())
		{
			corrs[v].push_back(v.idx());
			originalPoints[v.idx()] = points[v];
		}
		isInitialized = true;
	}	

	double previousVolume = std::numeric_limits<double>::infinity();
	unsigned int iterationsWithLowVolumeVariation = 0;
	double minVolume = std::numeric_limits<double>::infinity();
	Point reference = mesh->bbox().center();	
	int i = 0;
	int n = 0;
	while (true)
	{
		if (abort->abort)
		{
			mesh->garbage_collection();
			return;
		}

		algorithm_iteration(originalPoints);
		
		//mesh->write("iteration" + std::to_string(n++) + ".off");
		
		/*if (fixedVertices == previousFixedVertices)
		{
			++iterationsWithEqualFixedVertices;
			if (iterationsWithEqualFixedVertices >= 5 && fixedVertices > 0)
				break;
		}
		else
			iterationsWithEqualFixedVertices = 0;
		if (fixedVertices >= 0.4 * mesh->n_vertices())
			break;
		previousFixedVertices = fixedVertices;*/

		double area = 0.0;
		double volume = 0.0;
		for (auto face : mesh->faces())
		{
			auto vertices = mesh->vertices(face);
			auto v1 = (Vertex)(vertices); ++vertices;
			auto v2 = (Vertex)(vertices); ++vertices;
			auto v3 = (Vertex)(vertices); ++vertices;
			
			auto p1 = points[v1];
			auto p2 = points[v2];
			auto p3 = points[v3];

			auto c = (p2 - p1).cross(p3 - p1);
			area += c.norm();
			volume += c.dot(p1 - reference);
		}
		double volChange = ((abs(previousVolume) - abs(volume)) / abs(previousVolume));
		std::cout << "Remaining area: " << area << ", ";
		std::cout << "Remaining volume: " << volume << "; ";
		std::cout << "Volume change: " << volChange << "; ";
		std::cout << "Vertices: " << mesh->n_vertices() << std::endl;

		if (std::isnan(volume))
		{
			mesh->write("error.off");
			throw;
		}

		/*if (abs(volChange) < 0.0001)
		{
			++iterationsWithLowVolumeVariation;
			if (iterationsWithLowVolumeVariation >= 5)
				break;
		}
		else
			iterationsWithLowVolumeVariation = 0;*/
		if (abs(area) < minVolume)
		{
			minVolume = abs(area);
			iterationsWithLowVolumeVariation = 0;
		}
		else
		{
			++iterationsWithLowVolumeVariation;
			if (iterationsWithLowVolumeVariation >= 50)
				break;
		}

		previousVolume = volume;

		if (i++ >= 20)
		{
			i = 0;
			mesh->garbage_collection();
		}
	}

	mesh->garbage_collection();
	mesh->write("after_contraction.off");
}

typedef WingedEdge::WingedEdgeMeshModel TSkeletonMesh;

void collapseEdge(TSkeletonMesh* skel, TSkeletonMesh::Edge& e, 
	ElementProperty<std::set<TSkeletonMesh::Vertex>, TSkeletonMesh::Vertex>&	vrecord)
{
	TSkeletonMesh::Vertex v1 = skel->vertex(e, 0); // 'v1' will be deleted
	TSkeletonMesh::Vertex v2 = skel->vertex(e, 1);

	/// Do collapse
	skel->collapse(e);

	// carry its records too 
	vrecord[v2].insert(vrecord[v1].begin(), vrecord[v1].end());
}

CurveSkeleton* Skelcollapse::collapseToSkeleton()
{	
	TSkeletonMesh* skel = new TSkeletonMesh();

	/// 0) Construct winged-edge mesh
	ElementProperty<SurfaceMeshModel::Point, Vertex> meshPoints = mesh->get_vertex_property<SurfaceMeshModel::Point>("v:point");
	for (Surface_mesh::Vertex_iterator vit = mesh->vertices_begin(); vit != mesh->vertices_end(); ++vit)
	{
		SurfaceMeshModel::Point p = meshPoints[vit];
		skel->add_vertex(SurfaceMeshModel::Vector3(p[0], p[1], p[2]));
	}
	ElementProperty<SurfaceMeshModel::Point, TSkeletonMesh::Vertex> 
		skelPoints = skel->get_vertex_property<SurfaceMeshModel::Point>("v:point");

	// faces
	for (Surface_mesh::Face_iterator fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit)
	{
		Surface_mesh::Vertex_around_face_circulator fvit = mesh->vertices(fit), fvend = fvit;
		std::vector<TSkeletonMesh::Vertex> vertices;

		do {
			int vi = Surface_mesh::Vertex(fvit).idx();
			vertices.push_back(TSkeletonMesh::Vertex(vi));
		} while (++fvit != fvend);

		skel->add_face(vertices);
	}

	/// 1) perform sorted edge collapse
	// Add to priority queue
	EdgePropertyPriorityQueue<TSkeletonMesh> queue(skel, ELENGTH);
	ElementProperty<float, TSkeletonMesh::Edge> elenght = skel->edge_property<float>(ELENGTH, 0.0f);
	for (TSkeletonMesh::Edge eit : skel->edges())
	{
		elenght[eit] = skel->edge_length(eit);
		queue.insert(eit, elenght[eit]);
	}	
		
	// This will be used to position collapsed vertices
	ElementProperty<std::set<TSkeletonMesh::Vertex>, TSkeletonMesh::Vertex>
		vrecord = skel->vertex_property< std::set< TSkeletonMesh::Vertex> >(VCOLLAPSEFROM);

	// First add yourself to the set
	for(auto v : skel->vertices())
		vrecord[v].insert(v);

	/// Collapse cycle	
	while (!queue.empty())
	{
		if (abort->abort)
		{
			mesh->garbage_collection();
			return nullptr;
		}

		/// Retrieve shortest edge
		TSkeletonMesh::Edge e = queue.pop();

		/// Make sure edge was not already dealt with by previous collapses
		if (!skel->has_faces(e) || skel->is_deleted(e) || !skel->is_valid(e))
			continue;
	
		collapseEdge(skel, e, vrecord);

		TSkeletonMesh::Vertex v2 = skel->vertex(e, 1);
		/// Update length of edges incident to remaining vertex
		TSkeletonMesh::Edge_around_vertex eit(skel, v2);
		while (!eit.end())
		{
			double newLength = skel->edge_length(e);

			// If edge still in queue, update its position
			if (queue.has(e))
				queue.update(e, newLength);
			++eit;
		}
	}
	
	//Remove vertices without correspondences
	/*bool removedAnyVertex;
	do
	{
		if (abort->abort)
		{
			mesh->garbage_collection();
			return nullptr;
		}

		removedAnyVertex = false;
		for (auto vit = skel->vertices_begin(); vit != skel->vertices_end(); ++vit)
		{
			TSkeletonMesh::Vertex v = vit;		
			bool hasCorrespondences = false;
			for (auto recIt = vrecord[vit].begin(); recIt != vrecord[vit].end(); ++recIt)
			{
				if (corrs[SurfaceMeshModel::Vertex(recIt->idx())].size() > 0)
				{
					hasCorrespondences = true;
					break;
				}
			}

			if (!hasCorrespondences)
			{

				//find the closest neighbor
				TSkeletonMesh::Edge shortestEdge;
				TSkeletonMesh::Scalar shortestEdgeLength = std::numeric_limits<TSkeletonMesh::Scalar>::max();
				for (auto e : skel->edges_of(v))
				{
					auto length = skel->edge_length(e);
					if (length < shortestEdgeLength)
					{
						shortestEdge = e;
						shortestEdgeLength = length;
					}
				}

				collapseEdge(skel, shortestEdge, vrecord);
				removedAnyVertex = true;
				break;
			}
		}
	} while (removedAnyVertex); //collapse might invalidate the iteration*/


	auto curveSkeleton = new CurveSkeleton;
	std::map<int, int> skelVertIdxToCurveSkelVertIdx;

	// Move to centroid of collapsed vertices
	for (auto vit = skel->vertices_begin(); vit != skel->vertices_end() ; ++vit)
	{
		const TSkeletonMesh::Vertex v = (TSkeletonMesh::Vertex)vit;
		
		CurveSkeleton::Vertex curveSkelVert;
		curveSkelVert.position = Eigen::Vector3f(0, 0, 0);

		// Center of collapsed from vertices
		for(auto vn : vrecord[v])
		{
			auto smv = SurfaceMeshModel::Vertex(vn.idx());
			SurfaceMeshModel::Point p = meshPoints[smv];
			curveSkelVert.position += Eigen::Vector3f(p[0], p[1], p[2]);

			for (auto it = corrs[smv].begin(); it != corrs[smv].end(); ++it)
				curveSkelVert.correspondingOriginalVertices.push_back(*it);
		}

		curveSkelVert.position /= vrecord[v].size();
		curveSkeleton->vertices.push_back(curveSkelVert);
		skelVertIdxToCurveSkelVertIdx[((TSkeletonMesh::Vertex)vit).idx()] = curveSkeleton->vertices.size() - 1;
	}

	for (auto eit = skel->edges_begin(); eit != skel->edges_end(); ++eit)
	{
		//check if we need to subdivide the edge
		int i1 = skelVertIdxToCurveSkelVertIdx[skel->vertex(eit, 0).idx()];  // curve skeleton idx
		int i2 = skelVertIdxToCurveSkelVertIdx[skel->vertex(eit, 1).idx()];  // curve skeleton idx

		auto& v1 = curveSkeleton->vertices.at(i1);
		auto& v2 = curveSkeleton->vertices.at(i2);

		float edgeLength = (v1.position - v2.position).norm();
		int subdivisions = (int)round(edgeLength / (5 * edgelength_TH));
		if (subdivisions > 1)
		{
			std::vector<CurveSkeleton::Vertex*> relevantVertices;
			relevantVertices.reserve(subdivisions + 1);
			relevantVertices.push_back(&v1);

			//calculate positions for the new vertices
			for (int i = 1; i < subdivisions; ++i)
			{
				curveSkeleton->vertices.push_back(CurveSkeleton::Vertex((1 - (double)i / subdivisions) * v1.position + ((double)i / subdivisions) * v2.position));
				relevantVertices.push_back(&curveSkeleton->vertices.back());
			}

			relevantVertices.push_back(&v2);

			//insert new edges
			curveSkeleton->edges.emplace_back(i1, curveSkeleton->vertices.size() - (subdivisions - 1));
			for(int i = 0; i < subdivisions - 2; ++i)
				curveSkeleton->edges.emplace_back(curveSkeleton->vertices.size() - (subdivisions - 1) + i, curveSkeleton->vertices.size() - (subdivisions - 1) + i + 1);
			curveSkeleton->edges.emplace_back(curveSkeleton->vertices.size() - 1, i2);


			//update correspondences
			std::vector<int> corrs; corrs.reserve(v1.correspondingOriginalVertices.size() + v2.correspondingOriginalVertices.size());
			corrs.insert(corrs.end(), v1.correspondingOriginalVertices.begin(), v1.correspondingOriginalVertices.end());
			corrs.insert(corrs.end(), v2.correspondingOriginalVertices.begin(), v2.correspondingOriginalVertices.end());
			v1.correspondingOriginalVertices.clear();
			v2.correspondingOriginalVertices.clear();

			auto dir = (v2.position - v1.position) / subdivisions;
			auto projector = dir / dir.squaredNorm();

			for (int c : corrs)
			{
				auto p = originalPoints.at(c).cast<float>();
				float localCoord = (p - v1.position).dot(projector);
				int localIdx = std::max(0, std::min((int)round(localCoord), subdivisions));
				relevantVertices.at(localIdx)->correspondingOriginalVertices.push_back(c);
			}
		}
		else
		{
			curveSkeleton->edges.emplace_back(i1, i2);
		}
	}

	/// now, delete the items that have been marked to be deleted
	mesh->garbage_collection();

	return curveSkeleton;	
}