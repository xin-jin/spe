/**
 * @file poisson.cpp
 * Test script for treating the Graph as a MTL Matrix
 * and solving a Poisson equation.
 *
 * @brief Reads in two files specified on the command line.
 * First file: 3D Points (one per line) defined by three doubles.
 * Second file: Eges (one per line) defined by 2 indices into the point list
 *              of the first file.
 *
 * Launches an SDLViewer to visualize the solution.
 */

#include <fstream>
#include <cmath>

#include "CME212/SDLViewer.hpp"
#include "CME212/Util.hpp"
#include "CME212/Point.hpp"
#include "CME212/BoundingBox.hpp"

#include "Graph.hpp"

struct Force {
    double operator()(const Point& x) {
        return std::cos(norm_1(x))*5;
    }
};

struct Boundary {
    double operator()(const Point& x) {
        Box3D box(Point(-.6, -.2, -1), Point(.6, .2, 1));
        if (norm_inf(x-Point(.6, .6, 0)) < .2 &&
            norm_inf(x-Point(-.6, .6, 0)) < .2 &&
            norm_inf(x-Point(.6, -.6, 0)) < .2 &&
            norm_inf(x-Point(-.6, -.6, 0)) < .2)
            return -.2;
        else if (box.contains(x))
            return 1;
        else
            return 0;
    }
};


// HW3: YOUR CODE HERE
// Define a GraphSymmetricMatrix that maps
// your Graph concept to MTL's Matrix concept. This shouldn't need to copy or
// modify Graph at all!
typedef Graph<char,char> GraphType;  //<  DUMMY Placeholder

/** Remove all the nodes in graph @a g whose position is within Box3D @a bb.
 * @post For all i, 0 <= i < @a g.num_nodes(),
 *        not bb.contains(g.node(i).position())
 */
void remove_box(GraphType& g, const Box3D& bb) {
	for (auto n : nodesRange(g)) {
		if (bb.contains(n.position())) {
			g.remove_node(n);
		}
	}
    return;
}



int main(int argc, char** argv)
{
    // Check arguments
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " NODES_FILE TETS_FILE\n";
        exit(1);
    }

    // Define an empty Graph
    GraphType graph;

    // Create a nodes_file from the first input argument
    std::ifstream nodes_file(argv[1]);
    // Interpret each line of the nodes_file as a 3D Point and add to the Graph
    std::vector<typename GraphType::node_type> node_vec;
    Point p;
    while (CME212::getline_parsed(nodes_file, p))
        node_vec.push_back(graph.add_node(2*p - Point(1,1,0)));

    // Create a tets_file from the second input argument
    std::ifstream tets_file(argv[2]);
    // Interpret each line of the tets_file as four ints which refer to nodes
    std::array<int,4> t;
    while (CME212::getline_parsed(tets_file, t)) {
        graph.add_edge(node_vec[t[0]], node_vec[t[1]]);
        graph.add_edge(node_vec[t[0]], node_vec[t[2]]);
        graph.add_edge(node_vec[t[1]], node_vec[t[3]]);
        graph.add_edge(node_vec[t[2]], node_vec[t[3]]);
    }

    // Get the edge length, should be the same for each edge
    auto it = graph.edge_begin();
    assert(it != graph.edge_end());
    double h = norm((*it).node1().position() - (*it).node2().position());

    // Make holes in our Graph
    remove_box(graph, Box3D(Point(-0.8+h,-0.8+h,-1), Point(-0.4-h,-0.4-h,1)));
    remove_box(graph, Box3D(Point( 0.4+h,-0.8+h,-1), Point( 0.8-h,-0.4-h,1)));
    remove_box(graph, Box3D(Point(-0.8+h, 0.4+h,-1), Point(-0.4-h, 0.8-h,1)));
    remove_box(graph, Box3D(Point( 0.4+h, 0.4+h,-1), Point( 0.8-h, 0.8-h,1)));
    remove_box(graph, Box3D(Point(-0.6+h,-0.2+h,-1), Point( 0.6-h, 0.2-h,1)));

    // HW3: YOUR CODE HERE
    // Define b using the graph, f, and g.
    // Construct the GraphSymmetricMatrix A using the graph
    // Solve Au = b using MTL.

    return 0;
}
