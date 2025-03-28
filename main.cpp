#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <iostream>
#include <unordered_map>

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Simple_cartesian<double> 						Kernel;
typedef Kernel::Point_3 							Point;
typedef CGAL::Surface_mesh<Point> 						SurfaceMesh;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>		Polyhedron;

// AABB Tree setup
typedef CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh> 			AABB_Primitive;
typedef CGAL::AABB_traits<Kernel, AABB_Primitive> 				AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> 						AABB_Tree;

// Skeleton Setup
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor          	 	vertex_descriptor;
typedef CGAL::Mean_curvature_flow_skeletonization<SurfaceMesh>			Skeletonization;
typedef Skeletonization::Skeleton						Skeleton;
typedef Skeleton::vertex_descriptor						Skeleton_vertex;

// Threshold for thin walls
const double THIN_WALL_THRESHOLD = 0.05;  // Adjust based on mesh scale

int main(int argc, char* argv[]) {
    // Check input
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " mesh_file.off\n";
        return EXIT_FAILURE;
    }

    // Load the mesh
    SurfaceMesh mesh;
    if (!PMP::IO::read_polygon_mesh(argv[1], mesh)) {
        std::cerr << "Error: Cannot read mesh file " << argv[1] << "\n";
        return EXIT_FAILURE;
    }
    
    std::cout << "Mesh loaded: " << num_vertices(mesh) << " vertices, " << num_faces(mesh) << " faces\n";

    // Build the AABB tree
    AABB_Tree mesh_tree(faces(mesh).first, faces(mesh).second, mesh);
    mesh_tree.accelerate_distance_queries();  // Optimize for nearest point queries
    
    SurfaceMesh original_mesh;
    original_mesh = mesh;

    CGAL::Polygon_mesh_processing::isotropic_remeshing(mesh.faces(), 0.01, mesh); 

    /*Skeleton skeleton;
    Skeletonization skeletonization(mesh);
    //skeletonization.set_area_variation_factor(0.001); 
    //skeletonization.set_max_iterations(10); 
    //skeletonization.set_quality_speed_tradeoff(0.6); 
    skeletonization(skeleton);
    //CGAL::extract_mean_curvature_flow_skeleton(mesh, skeleton);

    for (Skeleton_vertex v : CGAL::make_range(vertices(skeleton))) {
        Point skeleton_point = skeleton[v].point;

        // Find the nearest mesh point from the skeleton
	Point nearest = mesh_tree.closest_point(skeleton_point);
        double thickness = CGAL::squared_distance(skeleton_point, nearest);

        // Assign colors
        //if (thickness < THIN_WALL_THRESHOLD * THIN_WALL_THRESHOLD) {
	std::cout << "Vertex: (" << skeleton_point << ") is associated to:" << std::endl;
	for (vertex_descriptor vd : skeleton[v].vertices) {
	    std::cout << "\t" << get(CGAL::vertex_point, mesh, vd) << std::endl;
	}
        //}
    }
*/
    // Draw the mesh with vertex coloring
    CGAL::draw(original_mesh);
    CGAL::draw(mesh);

    return EXIT_SUCCESS;
}

