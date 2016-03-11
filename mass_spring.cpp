/**
 * @file mass_spring.cpp
 * Implementation of mass-spring system using Graph
 *
 * @brief Reads in two files specified on the command line.
 * First file: 3D Points (one per line) defined by three doubles
 * Second file: Tetrahedra (one per line) defined by 4 indices into the point
 * list
 */

#include <fstream>

#include "CME212/SDLViewer.hpp"
#include "CME212/Util.hpp"
#include "CME212/Color.hpp"
#include "CME212/Point.hpp"
#include "thrust/for_each.h"
#include "thrust/system/omp/execution_policy.h"

#include "graph/Graph.hpp"
#include "struct/combined_force.hpp"
#include "struct/combined_constraint.hpp"



// Gravity in meters/sec^2
static constexpr double grav = 9.81;

/** Custom structure of data to store with Nodes */
struct NodeData {
    Point vel;                  //< Node velocity
    double mass;                //< Node mass
};

/** Custom structure of data to store with Edges */
struct EdgeData {
    double K;                   // Spring constant
    double L;                   // Spring rest length
};

static constexpr double K = 100.0;

typedef Graph<NodeData, EdgeData> GraphType;
typedef typename GraphType::node_type NodeType;
typedef typename GraphType::edge_type EdgeType;

using   size_type = typename GraphType::size_type;

/** Change a graph's nodes according to a step of the symplectic Euler
 *    method with the given node force.
 * @param[in,out] g      Graph
 * @param[in]     t      The current time (useful for time-dependent forces)
 * @param[in]     dt     The time step
 * @param[in]     force  Function object defining the force per node
 * @return the next time step (usually @a t + @a dt)
 *
 * @tparam G::node_value_type supports ???????? YOU CHOOSE
 * @tparam F is a function object called as @a force(n, @a t),
 *           where n is a node of the graph and @a t is the current time.
 *           @a force must return a Point representing the force vector on Node
 *           at time @a t.
 * @tparam C is a function object called as @a force(@a g, @a t)
 *           it should reset all points violating the specified constraints.
 */
template <typename G, typename F, typename C>
double symp_euler_step(G& g, double t, double dt, F force, C constraint) {
	// Compute the t+dt position	
	thrust::for_each(thrust::omp::par, g.node_begin(), g.node_end(),
					 [dt](NodeType n){ n.position() += n.value().vel * dt; });		
	
	// Deprecated
    // for (auto it = g.node_begin(); it != g.node_end(); ++it) {
    //     auto n = *it;
    //     // Update the position of the node according to its velocity
    //     // x^{n+1} = x^{n} + v^{n} * dt
    //     n.position() += n.value().vel * dt;
    // }


    constraint(g, t);

    // Compute the t+dt velocity
	thrust::for_each(thrust::omp::par, g.node_begin(), g.node_end(),
					 [dt, force, t](NodeType n) mutable { n.value().vel += force(n, t) * (dt / n.value().mass); });
	
	// Deprecated
    // for (auto it = g.node_begin(); it != g.node_end(); ++it) {
    //     auto n = *it;
    //     // v^{n+1} = v^{n} + F(x^{n+1},t) * dt / m
    //     n.value().vel += force(n, t) * (dt / n.value().mass);
    // }

    return t + dt;
}


/** Force function object for HW2 #1. */
struct Problem1Force {
    /** Return the force applying to @a n at time @a t.
     *
     * For HW2 #1, this is a combination of mass-spring force and gravity,
     * except that points at (0, 0, 0) and (1, 0, 0) never move. We can
     * model that by returning a zero-valued force. */
    template <typename NODE>
    Point operator()(const NODE& n, double t) {
        Point &xi = n.position();
        (void) t;

        Point fSpring(0);
        for (auto j = n.edge_begin(); j != n.edge_end(); ++j) {
            Point &xj = j.node2().position();
            Point xDiff = xi - xj;
            double xDiffLen = norm(xDiff);
            EdgeData eValue = (*j).value();
            fSpring -= eValue.K*xDiff/xDiffLen*(xDiffLen - eValue.L);
        }

        Point fGrav(0, 0, -grav);
        fGrav *= n.value().mass;
        return fSpring + fGrav;
    }
    static double L;
};

/** Initialize NodeData */
void initNodes(GraphType& g) {
    double m = 1.0/g.num_nodes();
    for (NodeType&& n : nodesRange(g)) {
        n.value().vel = Point(0);
        n.value().mass = m;
    }
}

/** Initialize EdgeData */
void initEdges(GraphType& g) {
    for (EdgeType&& e : edgesRange(g)) {
        e.value().K = 100.0/g.num_nodes();
        e.value().L = e.length();
    }
}

/** Force due to mass spring */
struct MassSpringForce {
    template <typename Node>
    Point operator()(Node n, double t) {
        Point &xi = n.position();
        (void) t;

        Point fSpring(0);
        for (auto j = n.edge_begin(); j != n.edge_end(); ++j) {
            Point &xj = j.node2().position();
            Point xDiff = xi - xj;
            double xDiffLen = norm(xDiff);
            EdgeData eValue = (*j).value();
            fSpring -= eValue.K*xDiff/xDiffLen*(xDiffLen - eValue.L);
        }

        return fSpring;
    }
};

/** Force due to gravity */
struct GravityForce {
    template <typename Node>
    Point operator()(Node n, double t) {
        (void) t;
        return Point(0, 0, -grav*n.value().mass);
    }
};

/** Damping force */
struct DampingForce {
    template <typename Node>
    Point operator()(Node n, double t) {
        (void) t;
        return -c * n.value().vel;
    }

    static double c;
};

/** Fix certain nodes to their original positions
 *  This constraint requires the underlying nodes never be removed
 *  Time complexity: O(1)
 */
class FixedNodesConstraint {
public:
    FixedNodesConstraint() {};

    template <typename Graph>
    void operator()(Graph& graph, double t) {
        (void) t;
        for (size_type i{0}; i != nodes_.size(); ++i) {
            assert(graph.has_node(nodes_[i]));
            nodes_[i].position() = pt_[i];
        }
    }

    void operator+=(NodeType n) {
        nodes_.push_back(n);
        pt_.push_back(n.position());
    }

private:
    std::vector<NodeType> nodes_;
    std::vector<Point> pt_;
};

/** An invisible wall
 * Time complexity: O(graph.size())
 */
class PlaneConstraint {
public:
    PlaneConstraint() {}
    PlaneConstraint(double z, const Point&& pt): z_(z), pt_(pt) {}

    template <typename Graph>
    void operator()(Graph& graph, double t) {
        (void) t;
        for (NodeType&& n : nodesRange(graph)) {
            Point &xi = n.position();
            if (dot(xi, pt_) < z_) {
                xi.z = z_;
                n.value().vel.z = 0;
            }
        }
    }

private:
    double z_ = -0.75;
    Point pt_ = Point(0, 0, 1);
};

/** An invisible sphere
 * Time complexity: O(graph.size())
 */
class SphereConstraint {
public:
    SphereConstraint() {}
    SphereConstraint(const Point& c, double r): c_(c), r_(r) {}

    template <typename Graph>
    void operator()(Graph& graph, double t) {
        (void) t;
        for (NodeType&& n : nodesRange(graph)) {
            Point &xi = n.position();
            double dis = norm(xi - c_);
            if (dis < r_) {
                Point R = (xi-c_)/dis;
                Point &v = n.value().vel;
                xi = R*r_ + c_;
                v = v - dot(v, R)*R;
            }
        }
    }

private:
    Point c_ = Point(.5, .5, -.5);
    double r_ = .15;
};

/** An invisible sphere. It eats everything that touches it (┛`д´)┛
 * Time complexity: O(graph.size())
 */
class DemonSphereConstraint {
public:
    DemonSphereConstraint() {}
    DemonSphereConstraint(const Point& c, double r): c_(c), r_(r) {}

    template <typename Graph>
    void operator()(Graph& graph, double t) {
        (void) t;
        for (NodeType&& n : nodesRange(graph)) {
            Point &xi = n.position();
            double dis = norm(xi - c_);
            if (dis < r_) {
                graph.remove_node(n);
            }
        }
    }

private:
    Point c_ = Point(.5, .5, -.5);
    double r_ = .15;
};


double Problem1Force::L;
double DampingForce::c;

int main(int argc, char** argv) {
    // Check arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " NODES_FILE TETS_FILE\n";
        exit(1);
    }

    // Construct an empty graph
    GraphType graph;

    // Create a nodes_file from the first input argument
    std::ifstream nodes_file(argv[1]);
    // Interpret each line of the nodes_file as a 3D Point and add to the Graph
    Point p;
    std::vector<typename GraphType::node_type> nodes;
    while (CME212::getline_parsed(nodes_file, p))
        nodes.push_back(graph.add_node(p));

    // Create a tets_file from the second input argument
    std::ifstream tets_file(argv[2]);
    // Interpret each line of the tets_file as four ints which refer to nodes
    std::array<int,4> t;
    while (CME212::getline_parsed(tets_file, t)) {
        graph.add_edge(nodes[t[0]], nodes[t[1]]);
        graph.add_edge(nodes[t[0]], nodes[t[2]]);
        // Diagonal edges
        graph.add_edge(nodes[t[0]], nodes[t[3]]);
        graph.add_edge(nodes[t[1]], nodes[t[2]]);

        graph.add_edge(nodes[t[1]], nodes[t[3]]);
        graph.add_edge(nodes[t[2]], nodes[t[3]]);
    }

    // Set initial conditions
    initNodes(graph);
    initEdges(graph);
    Problem1Force::L = (*(graph.edge_begin())).length();
    DampingForce::c = 1.0/graph.num_nodes();

    // Print out the stats
    std::cout << graph.num_nodes() << " " << graph.num_edges() << std::endl;

    // Launch the SDLViewer
    CME212::SDLViewer viewer;
    auto node_map = viewer.empty_node_map(graph);
    viewer.launch();

    viewer.add_nodes(graph.node_begin(), graph.node_end(), node_map);
    viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);
    viewer.center_view();

    // Begin the mass-spring simulation
    double dt = 0.001;
    double t_start = 0;
    double t_end = 5.0;

    auto customForce = makeCombinedForce(GravityForce(), MassSpringForce(), DampingForce());
    FixedNodesConstraint fixedC;
    for (NodeType&& n : nodesRange(graph)) {
        if (n.position() == Point(0, 0, 0) || n.position() == Point(1, 0, 0))
            fixedC += n;
    }
    auto customConstraint = makeCombinedConstraint(fixedC, PlaneConstraint(), SphereConstraint());

    for (double t = t_start; t < t_end; t += dt) {
        //std::cout << "t = " << t << std::endl;
        symp_euler_step(graph, t, dt, customForce, customConstraint);

        // Clear the viewer's nodes and edges
        viewer.clear();
        node_map.clear();

        // Update viewer with nodes' new positions
        viewer.add_nodes(graph.node_begin(), graph.node_end(), node_map);
        viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);
        viewer.set_label(t);

        // These lines slow down the animation for small graphs, like grid0_*.
        // Feel free to remove them or tweak the constants.
		if (graph.size() < 100)
            CME212::sleep(0.001);
    }

    return 0;
}
