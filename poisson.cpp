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
#include "boost/numeric/mtl/mtl.hpp"
#include <boost/numeric/itl/itl.hpp>

#include "Graph.hpp"

typedef Graph<bool,char> GraphType;  //<  DUMMY Placeholder
using NodeType = GraphType::Node;
using VecType = mtl::dense_vector<double>;
double tol = 1e-10;

struct Force {
    double operator()(const Point& x) {
        return std::cos(norm_1(x))*5.0;
    }
};

struct Boundary {
    static Box3D box;
    double operator()(const Point& x) const {
        if (norm_inf(x-Point(.6, .6, 0)) < .2   ||
            norm_inf(x-Point(-.6, .6, 0)) < .2  ||
            norm_inf(x-Point(.6, -.6, 0)) < .2  ||
            norm_inf(x-Point(-.6, -.6, 0)) < .2)
            return -.2;
        else if (box.contains(x))
            return 1;
        else
            return 0;
    }
    bool contains(const Point& x) const {
        if (norm_inf(x-Point(.6, .6, 0)) < .2   ||
            norm_inf(x-Point(-.6, .6, 0)) < .2  ||
            norm_inf(x-Point(.6, -.6, 0)) < .2  ||
            norm_inf(x-Point(-.6, -.6, 0)) < .2 ||
            box.contains(x)                     ||
            std::abs(norm_inf(x)-1) < tol)
            return true;
        else
            return false;
    }
};

class GraphSymmetricMatrix {
public:
    GraphSymmetricMatrix(const GraphType& g):
        graph_(g) {}

    size_t size() const {
        return graph_.size();
    }

    template <typename VectorIn, typename VectorOut, typename Assign>
    void mult(const VectorIn& v, VectorOut& w, Assign) const {
        assert(mtl::size(v) == mtl::size(w));
        assert(mtl::size(v) == size());
        for (size_t i = 0; i < size(); ++i) {
            if (graph_.node(i).value()) {
                Assign::apply(w[i], v[i]);
            }
            else {
                NodeType n = graph_.node(i);
                double tmp = -static_cast<double>(n.degree())*v[i];
                for (auto k = n.edge_begin(); k != n.edge_end(); ++k) {
                    if (!k.node2().value()) {
                        tmp += v[k.index()];
                    }
                }
                Assign::apply(w[i], tmp);
            }
        }
    }

    /** Matvec forwards to MTL's lazy mat_cvec_multiplier operator */
    template <typename Vector>
    typename mtl::vec::mat_cvec_multiplier<GraphSymmetricMatrix, Vector>
    operator*(const Vector& v) const {
        return {*this, v};
    }

private:
    const GraphType &graph_;
};

/** The number of elements in the matrix */
inline size_t size(const GraphSymmetricMatrix& A) {
    return A.size();
}
/** The number of rows in the matrix */
inline size_t num_rows(const GraphSymmetricMatrix& A) {
    return A.size();
}
/** The number of columns in the matrix */
inline size_t num_cols(const GraphSymmetricMatrix& A) {
    return A.size();
}

/** Traits that MTL uses to determine properties of our IdentityMatrix */
namespace mtl {
    namespace ashape {
        /** Define Identity Matrix to be a non-scalar type */
        template<>
        struct ashape_aux<GraphSymmetricMatrix> {
            typedef nonscal type;
        };
    }

    /** IdentityMatrix implements the Collection concept with value_type and size_type */
    template<>
    struct Collection<GraphSymmetricMatrix> {
        typedef double      value_type;
        typedef unsigned    size_type;
    };
}



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

Box3D Boundary::box(Point(-.6, -.2, -1), Point(.6, .2, 1));
Boundary boundary;

class PositionFunc {
public:
    PositionFunc(const VecType& u): u_(u) {}

    Point operator()(const NodeType& n) const {
        return {n.position().x, n.position().y, u_[n.index()]};
    }
private:
    const VecType &u_;
};

class ColorFunc {
public:
    ColorFunc(const VecType& u): u_(u) {};

    CME212::Color operator()(const NodeType& n) const {
        return CME212::Color::make_heat(std::abs(u_[n.index()])/(u_max+.0001));
    }

    void update_max() {
        u_max = std::abs(*(std::max_element(u_.begin(), u_.end(),
											[](double a, double b){ return std::abs(a) < std::abs(b); })));
    }
    
private:
	double u_max;
    const VecType &u_;
};

template <class Real, class OStream = std::ostream>
class visual_iteration: public itl::cyclic_iteration<Real> {
public:
	typedef itl::cyclic_iteration<Real> super;
	
    template <typename Vector>
    visual_iteration(ColorFunc& cf, const PositionFunc& pf, const GraphType& graph,
                     const Vector& r0, int max_iter_, Real tol_,
                     Real atol_ = Real(0), int cycle_ = 100, OStream& out = std::cout):
		super(r0, max_iter_, tol_, atol_, cycle_, out), 
        cf_(cf), pf_(pf), graph_(graph) {
        viewer_.launch();
		node_map_ = viewer_.empty_node_map(graph_);
    }

    bool finished() {
        return super::finished();
    }

    template <typename T>
    bool finished(const T& r) {
		CME212::sleep(.1);
        cf_.update_max();
        viewer_.add_nodes(graph_.node_begin(), graph_.node_end(), cf_, pf_, node_map_);
        viewer_.add_edges(graph_.edge_begin(), graph_.edge_end(), node_map_);
        return super::finished(r);
    }

private:
    ColorFunc &cf_;
    const PositionFunc &pf_;
    CME212::SDLViewer viewer_;
    const GraphType &graph_;
	decltype(viewer_.empty_node_map(graph_)) node_map_;
};

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

    // n.value() == true iff node n is on the boundary
    Boundary bd;
    // Define Bd_;
    for (auto n : nodesRange(graph)) {
        if (bd.contains(n.position()))
			n.value() = true;
        else
			n.value() = false;
    }

    // Define b
    VecType b(graph.size(), 0.0);
    Force f;
    for (auto n : nodesRange(graph)) {
        if (n.value()) {
            b[n.index()] = bd(n.position());
        }
        else {
            b[n.index()] = h*h*f(n.position());
            for (auto e = n.edge_begin(); e != n.edge_end(); ++e) {
                b[n.index()] -= bd(e.node2().position());
            }
        }
    }

    // Define A
    GraphSymmetricMatrix A(graph);

    /////////////////////////////////////////
    // Solve discretized Poission equation //
    /////////////////////////////////////////
	VecType u(graph.size(), 0.0);
    // preconditioner
    itl::pc::identity<GraphSymmetricMatrix> L(A);
	ColorFunc cf(u);
	PositionFunc pf(u);
    // iteration object	
    visual_iteration<double> iter(cf, pf, graph, b, 500, tol, 0.0, 1);
    // Solve Au = b using conjugate gradient method
    itl::cg(A, u, b, L, iter);

    return 0;
}
