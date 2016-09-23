#pragma once

#include <set>

#include "surface_mesh/Surface_mesh.h"

/// @defgroup surfacemesh SurfaceMesh
namespace SurfaceMesh{

	/// @defgroup surfacemesh_basic_types Basic types
	/// @ingroup surfacemesh
	/// @{ 
	typedef Surface_mesh::Scalar    Scalar;              ///< Scalar type
	typedef Eigen::Vector3d         Vector3;             ///< 3D Vector type
	typedef Eigen::Vector3d         Point;               ///< Point type
	typedef Eigen::Vector3d         Normal;              ///< Point type
	typedef Eigen::Vector3d         Color;               ///< Point type
	typedef int                     Integer;             ///< int
	typedef unsigned int            Counter;             ///< To count stuff
	typedef unsigned int            Size;                ///< @obsolete To index stuff (i.e. matlab pointer) 
	// inherited from Surface_mesh
	typedef Surface_mesh::Edge                      Edge;
	typedef Surface_mesh::Halfedge                  Halfedge;
	typedef Surface_mesh::Vertex                    Vertex;
	typedef Surface_mesh::Face                      Face;
	/// @}

	/// @defgroup surfacemesh_compatibility_types Back-compatibility types
	/// @ingroup surfacemesh
	/// @{
	// these are to avoid problems from conversion from Surface_mesh::Vector
	template <class T> Scalar dot(const T& a, const T&b){ return a.dot(b); }
	template <class T> T cross(const T& a, const T&b){ return a.cross(b); }
	typedef Eigen::Vector3d Vec3d;
	typedef Eigen::Vector4d Vec4d;
	typedef Eigen::Vector3i Vec3i;
	typedef Eigen::Vector3f Vec3f;
	typedef Eigen::Vector2f Vec2f;
	/// @}

	/// @defgroup surfacemesh_property_names Default property names
	/// The std::string constants you should use to access SurfaceMesh dynamic properties.
	/// For example to obtain the coordinate property you should use: <br>
	/// @code
	/// Vector3VertexProperty points = mesh->vertex_property<Vector3>(VPOINT);
	/// @endcode
	/// 
	/// @ingroup surfacemesh
	/// @{ 
	const std::string VPOINT = "v:point";           ///< vertex coordinates
	const std::string VNORMAL = "v:normal";         ///< vertex normals
	const std::string VCOLOR = "v:color";           ///< vertex color
	const std::string VAREA = "v:area";             ///< vertex areas
	const std::string VQUALITY = "v:quality";       ///< vertex quality
	const std::string VCOLLAPSEFROM = "v:collapse-from";
	const std::string FNORMAL = "f:normal";         ///< face normals
	const std::string FAREA = "f:area";             ///< face area
	const std::string ELENGTH = "e:length";         ///< edge length
	const std::string FSELECTED = "f:selected";     ///< is face selected?    	
	/// @}

	/// @defgroup surfacemesh_property_types Default property types
	/// Some default property types, for example Surface_mesh::Vertex_property<Scalar> becomes ScalarVertexProperty
	/// @ingroup surfacemesh
	/// @{
	// Default Vertex properties
	typedef ElementProperty<Scalar, Vertex>   ScalarVertexProperty;   ///< A scalar associated to a vertex.
	typedef ElementProperty<Integer, Vertex>  IntegerVertexProperty;  ///< An (signed) integer number associated to a vertex.
	typedef ElementProperty<Vector3, Vertex>  Vector3VertexProperty;  ///< An Vector3 associated to a vertex.
	typedef ElementProperty<bool, Vertex>     BoolVertexProperty;     ///< A boolean associated to a vertex.
	typedef ElementProperty<std::set<Vertex>, Vertex> VerticesVertexProperty;
	// Default Face properties
	typedef ElementProperty<Scalar, Face>     ScalarFaceProperty;     ///< A scalar associated to a face.
	typedef ElementProperty<Vector3, Face>    Vector3FaceProperty;    ///< A Vector3 associated to a face.
	typedef ElementProperty<bool, Face>       BoolFaceProperty;       ///< A boolean associated to a face.
	// Default Edge properties
	typedef ElementProperty<Scalar, Edge>     ScalarEdgeProperty;     ///< A scalar associated to an edge.
	typedef ElementProperty<bool, Edge>       BoolEdgeProperty;       ///< A boolean associated to an edge.
	// Default Halfedge properties
	typedef ElementProperty<Scalar, Halfedge> ScalarHalfedgeProperty; ///< A scalar associated to an halfedge.
	/// @}    

	/// @brief Forward declaration of helpers (reduces clutter of this class header)
	/// @{ 
	class SurfaceMeshForEachVertexOnFaceHelper;
	class SurfaceMeshForEachVertexHelper;
	class SurfaceMeshForEachEdgeHelper;
	class SurfaceMeshForEachOneRingEdgesHelper;
	class SurfaceMeshForEachFaceHelper;
	class SurfaceMeshForEachHalfedgeHelper;
	class SurfaceMeshForEachFaceAtVertex;
	/// @}     

	/**
	* @brief A starlab Model for the Surface_mesh datatype
	* @defgroup surfacemesh SurfaceMeshModel
	*/
	class SurfaceMeshModel : public Surface_mesh {

			/// @{ Basic Model Implementation
	public:
		void updateBoundingBox();
		/// @}

		/// @{ Qt foreach helpers
		///    Example: foreach(Vertex v, m->validVertices()){ ... }
	public:
		using Surface_mesh::halfedges; ///< Allows to use methods from Surface_mesh
		using Surface_mesh::faces;     ///< Allows to use methods from Surface_mesh
		using Surface_mesh::vertices;  ///< Allows to use methods from Surface_mesh

		/// @brief Returns iterator visiting every (valid) halfedge on a mesh
		SurfaceMeshForEachHalfedgeHelper halfedges();
		/// @brief Returns iterator visiting every (valid) edge on a mesh
		SurfaceMeshForEachEdgeHelper edges();
		/// @todo use the override scheme and just call it hedges(v)
		SurfaceMeshForEachOneRingEdgesHelper onering_hedges(Vertex v);
		/// @brief Returns iterator visiting every (valid) face on a mesh
		SurfaceMeshForEachFaceHelper faces();
		/// @brief Returns iterator visiting every face incident to a vertex
		/// @note Transparently overrides Surface_mesh's method but supports Qt::foreach
		SurfaceMeshForEachFaceAtVertex faces(Vertex v);
		/// @brief Returns iterator visiting every (valid) mesh vertex
		SurfaceMeshForEachVertexHelper vertices();
		/// @brief Returns iterator visiting every mesh vertex on a given face
		/// @note Transparently overrides Surface_mesh's method but supports Qt::foreach
		SurfaceMeshForEachVertexOnFaceHelper vertices(Face f);
		/// @}

		/// @{ Query existence of basic properties
		bool has_vertex_normals(){ return has_vertex_property<Vector3>(VNORMAL); }
		bool has_face_normals(){ return has_face_property<Vector3>(FNORMAL); }
		/// @}

		/// @{ Access to default properties
		Vector3VertexProperty vertex_coordinates(bool create_if_missing = false);
		Vector3VertexProperty vertex_normals(bool create_if_missing = false);
		Vector3FaceProperty   face_normals(bool create_if_missing = false);
		/// @}

		/// @{ forced garbage collection!!
		void garbage_collection(){ garbage_ = true; Surface_mesh::garbage_collection(); }
		/// @}

		/// @{ Extra exposed functionality
		/// @brief Removes vertex
		void remove_vertex(Vertex v);
		/// @}

		const Eigen::AlignedBox3d& bbox(){ return _bbox; }

	protected:
		/// Per-model bounding box (before transform is applied)
		Eigen::AlignedBox3d _bbox;
	};

} // namespace

#include "SurfaceMeshForEachHelpers.h"
#include "StdIterators.h"

/// Append namespace to name
