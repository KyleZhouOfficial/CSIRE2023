#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/boost/graph/IO/OBJ.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point Point;



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

int main(int argc, char* argv[])
{
    std::vector<Point> vertices;
    std::vector<Point> constraints;
    std::vector<Point> holes;
    std::vector<Vertex_handle> vertexHandles;

    //std::string polyFilename = "/Users/kylezhou/cgal/CGAL-5.5.2/data/A.poly";
    std::string polyFilename = argv[1];
    readPolyFile(polyFilename, vertices, constraints, holes);

    CDT cdt;

    // Insert vertices into CDT
    for (const auto& vertex : vertices)
    {
        Vertex_handle va = cdt.insert(vertex);
        vertexHandles.push_back(va);
    }

    // Insert constraints into CDT
    for (const auto& constraint : constraints)
    {
        cdt.insert_constraint(vertexHandles[constraint.x()], vertexHandles[constraint.y()]);
    }

    std::cout << "Meshing the domain..." << std::endl;
    CGAL::refine_Delaunay_mesh_2(cdt, holes.begin(), holes.end(),
                                 Criteria());


    // Perform any further operations or computations with the CDT
    //std::string outputFile = "/Users/kylezhou/cgal/CGAL-5.5.2/polyToObj/output.obj";
    std::string outputFile = argv[2];
    std::ofstream objFile(outputFile);
    if (!objFile.is_open()) {
        std::cerr << "Failed to open OBJ file for writing: " << outputFile << std::endl;
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
        auto v1 = vertexIndices[face->vertex(0)];
        auto v2 = vertexIndices[face->vertex(1)];
        auto v3 = vertexIndices[face->vertex(2)];
        objFile << "f " << v1 << " " << v2 << " " << v3 << std::endl;
    }

    objFile.close();

    return 0;
}

