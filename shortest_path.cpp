/**
 * @file shortest_path.cpp
 * Test script for using our templated Graph to determine shortest paths.
 *
 * @brief Reads in two files specified on the command line.
 * First file: 3D Points (one per line) defined by three doubles
 * Second file: Tetrahedra (one per line) defined by 4 indices into the point
 * list
 */

#include <vector>
#include <fstream>
#include <algorithm>
#include <queue>
#include <cassert>

#include "CME212/SDLViewer.hpp"
#include "CME212/Util.hpp"
#include "CME212/Color.hpp"

#include "Graph.hpp"


/** Comparator that compares the distance from a given point p.
 */
struct MyComparator {
    Point p_;
    MyComparator(const Point& p) : p_(p) {
    };

    template <typename NODE>
    bool operator()(const NODE& node1, const NODE& node2) const {
        auto d1 = norm(node1.position() - p_), d2 = norm(node2.position() - p_);
        return d1 < d2;
    }
};


/** Calculate shortest path lengths in @a g from the nearest node to @a point.
 * @param[in,out] g Input graph
 * @param[in] point Point to find the nearest node to.
 * @post Graph has modified node values indicating the minimum path length
 *           to the nearest node to @a point
 * @post Graph nodes that are unreachable to the nearest node to @a point have
 *           the value() -1.
 * @return The maximum path length found.
 *
 * Finds the nearest node to @a point and treats that as the root node for a
 * breadth first search.
 * This sets node's value() to the length of the shortest path to
 * the root node. The root's value() is 0. Nodes unreachable from
 * the root have value() -1.
 */
int shortest_path_lengths(Graph<int>& g, const Point& point) {
    using size_type = typename Graph<int>::size_type;
    using Node = typename Graph<int>::Node;

    // Find the closet Node to @a point
    auto root_iter = std::min_element(g.node_begin(), g.node_end(), MyComparator(point));
    // Initialize all values to be -1
	for (auto n : nodesRange(g)) {
		n.value() = -1;
	}
    (*root_iter).value() = 0;
    std::queue<Node> pQueue;
    pQueue.push(*root_iter);
    int maxLen{0};

    // Begin BFS
    while (!pQueue.empty()) {
        Node current = pQueue.front();
		pQueue.pop();				
        maxLen = current.value();
        for (auto i = current.edge_begin(); i != current.edge_end(); ++i) {
            Node node2 = i.node2();
            if (node2.value() == -1) {
                node2.value() = current.value() + 1;
                pQueue.push(node2);
            }
        }
    }

    return maxLen;
}

/** Customized color functor */
struct MyColorFunc {
    using Node = Graph<int>::Node;
    static int max;
    CME212::Color operator()(const Node& n) {
        if (n.value() == -1)
            return CME212::Color(0);
        else {
            float r = n.value()*1.0f/max;
            return CME212::Color::make_heat(1.0-r);
        }
    }
};

int MyColorFunc::max = 0;



int main(int argc, char** argv)
{
    // Check arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " NODES_FILE TETS_FILE\n";
        exit(1);
    }

    // Construct a Graph
    typedef Graph<int> GraphType;
    GraphType graph;
    std::vector<GraphType::node_type> nodes;

    // Create a nodes_file from the first input argument
    std::ifstream nodes_file(argv[1]);
    // Interpret each line of the nodes_file as a 3D Point and add to the Graph
    Point p;
    while (CME212::getline_parsed(nodes_file, p))
        nodes.push_back(graph.add_node(p));

    // Create a tets_file from the second input argument
    std::ifstream tets_file(argv[2]);
    // Interpret each line of the tets_file as four ints which refer to nodes
    std::array<int,4> t;
    while (CME212::getline_parsed(tets_file, t))
        for (unsigned i = 1; i < t.size(); ++i)
            for (unsigned j = 0; j < i; ++j)
                graph.add_edge(nodes[t[i]], nodes[t[j]]);

    // Print out the stats
    std::cout << graph.num_nodes() << " " << graph.num_edges() << std::endl;

    // Launch the SDLViewer
    CME212::SDLViewer viewer;
    viewer.launch();

    // Set the viewer
    MyColorFunc::max = shortest_path_lengths(graph, Point(-1, 0, 1));
    auto node_map = viewer.empty_node_map(graph);
    viewer.add_nodes(graph.node_begin(), graph.node_end(), MyColorFunc(), node_map);
	viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);
    viewer.center_view();

    return 0;
}
