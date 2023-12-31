

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <stdlib.h>

#define CGAL_MESH_2_OPTIMIZER_VERBOSE

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/lloyd_optimize_mesh_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

struct FaceInfo2
{
    FaceInfo2(){}
    int nesting_level;
    bool in_domain(){
        return nesting_level%2 == 1;
    }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_mesh_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag>  CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point Point;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;
typedef CGAL::Polygon_2<K>   Polygon_2;
typedef CDT::Face_handle  Face_handle;


//void
//mark_domains(CDT& ct,
//             Face_handle start,
//             int index,
//             std::list<CDT::Edge>& border )
//{
//    if(start->info().nesting_level != -1){
//        return;
//    }
//    std::list<Face_handle> queue;
//    queue.push_back(start);
//    while(! queue.empty()){
//        Face_handle fh = queue.front();
//        queue.pop_front();
//        if(fh->info().nesting_level == -1){
//            fh->info().nesting_level = index;
//            for(int i = 0; i < 3; i++){
//                CDT::Edge e(fh,i);
//                Face_handle n = fh->neighbor(i);
//                if(n->info().nesting_level == -1){
//                    if(ct.is_constrained(e)) border.push_back(e);
//                    else queue.push_back(n);
//                }
//            }
//        }
//    }
//}
////explore set of facets connected with non constrained edges,
////and attribute to each such set a nesting level.
////We start from facets incident to the infinite vertex, with a nesting
////level of 0. Then we recursively consider the non-explored facets incident
////to constrained edges bounding the former set and increase the nesting level by 1.
////Facets in the domain are those with an odd nesting level.
//void
//mark_domains(CDT& cdt)
//{
//    for(CDT::Face_handle f : cdt.all_face_handles()){
//        f->info().nesting_level = -1;
//    }
//    std::list<CDT::Edge> border;
//    mark_domains(cdt, cdt.infinite_face(), 0, border);
//    while(! border.empty()){
//        CDT::Edge e = border.front();
//        border.pop_front();
//        Face_handle n = e.first->neighbor(e.second);
//        if(n->info().nesting_level == -1){
//            mark_domains(cdt, n, e.first->info().nesting_level+1, border);
//        }
//    }
//}



void readPolyFile(const std::string& filename, std::vector<Point>& vertices, std::vector<Point>& constraints, std::vector<Point>& holes)
{
    std::ifstream inputFile(filename);
    if (!inputFile)
    {
        std::cerr << "Error opening input file: " << filename << std::endl;
        return;
    }

    int numVertices, numConstraints, vertexIndex, constraintIndex, dimensions, attributes, boundaryMarkers;
    double x, y;
    std::string line;
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        // Read number of vertices and constraints
        ss >> numVertices >> dimensions >> attributes >> boundaryMarkers;
        break;
    }
    std::cout << numVertices << dimensions << attributes << boundaryMarkers << std::endl;

    // Read vertices
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        for (int i = 0; i < numVertices; ++i) {
            if(i != 0) std::getline(inputFile, line);
            std::stringstream ss(line);
            ss >> vertexIndex >> x >> y;

            int a;
            for(int j = 0; j < attributes; j++) ss >> a;
            vertices.push_back(Point(x, y));
        }
        break;
    }

    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        int contraintBoundary;
        ss >> numConstraints >> contraintBoundary;
        // Read constraints
        break;
    }

    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        for (int i = 0; i < numConstraints; ++i) {
            if(i != 0) std::getline(inputFile, line);
            std::stringstream ss(line);
            ss >> constraintIndex >> x >> y;
            x--, y--;
            constraints.push_back(Point(x, y));
        }
        break;
    }

    int numHoles;
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        ss >> numHoles;
        break;
    }

    int holeIndex;
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        for (int i = 0; i < numHoles; i++) {
            if(i != 0) std::getline(inputFile, line);
            std::stringstream ss(line);
            ss >> holeIndex >> x >> y;
            holes.push_back(Point(x, y));
            std::cout << x << " " << y << std::endl;
        }
        break;
    }

    inputFile.close();
}

int main(int argc, char* argv[]) {
    std::vector<Point> vertices;
    std::vector<Point> constraints;
    std::vector<Point> holes;
    std::vector<Vertex_handle> vertexHandles;
//    argv[1] = "/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/data/part0.poly";
//    argv[2] = "26";
//    argv[3] = "0.1";
//    argv[4] = "2";
//    argv[5] = "3";
//    argv[6] = "/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/output/kyle.obj";

    //std::string polyFilename = "/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/data/part0.poly";
    std::string polyFilename = argv[1];
    readPolyFile(polyFilename, vertices, constraints, holes);

    CDT cdt;

    // Insert vertices into CDT
    for (const auto &vertex : vertices) {
        Vertex_handle va = cdt.insert(vertex);
        vertexHandles.push_back(va);
    }

    // Insert constraints into CDT
    for (const auto &constraint : constraints) {
        cdt.insert_constraint(vertexHandles[constraint.x()], vertexHandles[constraint.y()]);
    }

//    {
//    std::ofstream objFile("/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/output/test.obj");
//    if (!objFile.is_open()) {
//        std::cout << "Failed to open OBJ file for writing: " << std::endl;
//        return 0;
//    }
//
//    // Create a map to store the vertex indices
//    std::map<CDT::Vertex_handle, int> vertexIndices;
//
//    // Write vertices to the OBJ file
//    int index = 1; // Vertex index counter
//    for (auto vertex = cdt.finite_vertices_begin(); vertex != cdt.finite_vertices_end(); ++vertex) {
//        vertexIndices[vertex] = index;
//        objFile << "v " << vertex->point().x() << " " << vertex->point().y() << " 0" << std::endl;
//        ++index;
//    }
//    // Write faces to the OBJ file
//    for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
//            auto v1 = vertexIndices[face->vertex(0)];
//            auto v2 = vertexIndices[face->vertex(1)];
//            auto v3 = vertexIndices[face->vertex(2)];
//            objFile << "f " << v1 << " " << v2 << " " << v3 << std::endl;
//    }
//    std::cout << "Here "<< std::endl;
//
//    objFile.close();
//}

    //std::cout << atoi(argv[5]) << std::endl;
    /*
     * Shape Criteria: arcsin(1/2B) = 26 degrees
     * B = 1.14058601635
     * B = sqrt(1/4b)
     * b = 0.192169262338
     */
    double B = 1 / (2*sin(atof(argv[2]) * (M_PI/180)));
    double bInput = 1 / (pow(B, 2) * 4);

    for(int i = 0; i < atoi(argv[5]); i++) {
        CGAL::refine_Delaunay_mesh_2(cdt, holes.begin(), holes.end(),
                                     Criteria(bInput, atof(argv[3])));

        CGAL::lloyd_optimize_mesh_2(cdt, CGAL::parameters::max_iteration_number = atoi(argv[4]));
    }

    // Perform any further operations or computations with the CDT
    //std::string outputFile = "/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/output.obj";
    std::string outputFile = argv[6];
    std::ofstream objFile(outputFile);
    if (!objFile.is_open()) {
        std::cout << "Failed to open OBJ file for writing: " << outputFile << std::endl;
        return 0;
    }

    // Create a map to store the vertex indices
    std::map<CDT::Vertex_handle, int> vertexIndices;

    // Write vertices to the OBJ file
    int index = 1; // Vertex index counter
    for (auto vertex = cdt.finite_vertices_begin(); vertex != cdt.finite_vertices_end(); ++vertex) {
        vertexIndices[vertex] = index;
        objFile << "v " << vertex->point().x() << " " << vertex->point().y() << " 0" << std::endl;
        ++index;
    }


    // Write faces to the OBJ file
    for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
        if(face->is_in_domain()) {
            auto v1 = vertexIndices[face->vertex(0)];
            auto v2 = vertexIndices[face->vertex(1)];
            auto v3 = vertexIndices[face->vertex(2)];
            objFile << "f " << v1 << " " << v2 << " " << v3 << std::endl;
        }
    }
    std::cout << "Here " << outputFile << std::endl;

    objFile.close();

    return 0;
}

