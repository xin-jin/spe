#ifndef CME212_GRAPH_HPP
#define CME212_GRAPH_HPP

/** @file Graph.hpp
 * @brief An undirected graph type
 */

#include <algorithm>
#include <vector>
#include <cassert>

#include "CME212/Util.hpp"
#include "CME212/Point.hpp"


/** @class Graph
 * @brief A template for 3D undirected graphs.
 *
 * Users can add and retrieve nodes and edges. Edges are unique (there is at
 * most one edge between any pair of distinct nodes).
 */
class Graph {
private:

    // HW0: YOUR CODE HERE
    // Use this space for declarations of important internal types you need
    // later in the Graph's definition.
    // (As with all the "YOUR CODE HERE" markings, you may not actually NEED
    // code here. Just use the space if you need it.)

public:

    //
    // PUBLIC TYPE DEFINITIONS
    //

    /** Type of this graph. */
    typedef Graph graph_type;

    /** Predeclaration of Node type. */
    class Node;
    /** Synonym for Node (following STL conventions). */
    typedef Node node_type;

    /** Predeclaration of Edge type. */
    class Edge;
    /** Synonym for Edge (following STL conventions). */
    typedef Edge edge_type;

    /** Type of indexes and sizes.
        Return type of Graph::Node::index(), Graph::num_nodes(),
        Graph::num_edges(), and argument type of Graph::node(size_type) */
    typedef unsigned size_type;

    //
    // CONSTRUCTORS AND DESTRUCTOR
    //

    /** Construct an empty graph. */
    Graph(): points({}), edges({}) {
        // HW0: YOUR CODE HERE
    }

    /** Default destructor */
    ~Graph() = default;

    //
    // NODES
    //

    /** @class Graph::Node
     * @brief Class representing the graph's nodes.
     *
     * Node objects are used to access information about the Graph's nodes.
     */
    class Node {
    public:
        /** Construct an invalid node.
         *
         * Valid nodes are obtained from the Graph class, but it
         * is occasionally useful to declare an @i invalid node, and assign a
         * valid node to it later. For example:
         *
         * @code
         * Graph::node_type x;
         * if (...should pick the first node...)
         *   x = graph.node(0);
         * else
         *   x = some other node using a complicated calculation
         * do_something(x);
         * @endcode
         */
        Node() {
            // HW0: YOUR CODE HERE
        }

        /** Return this node's position. */
        const Point& position() const {
            // HW0: YOUR CODE HERE
            return graph_->points[index_];
        }

        /** Return this node's index, a number in the range [0, graph_size). */
        size_type index() const {
            // HW0: YOUR CODE HERE
            return index_;
        }

        /** Test whether this node and @a n are equal.
         *
         * Equal nodes have the same graph and the same index.
         */
        bool operator==(const Node& n) const {
            // HW0: YOUR CODE HERE
            (void) n;          // Quiet compiler warning
			if (graph_ == n.graph_ && index_ == n.index_)
				return true;
            else
				return false;
        }

        /** Test whether this node is less than @a n in a global order.
         *
         * This ordering function is useful for STL containers such as
         * std::map<>. It need not have any geometric meaning.
         *
         * The node ordering relation must obey trichotomy: For any two nodes x
         * and y, exactly one of x == y, x < y, and y < x is true.
         */
        bool operator<(const Node& n) const {
            // HW0: YOUR CODE HERE
            (void) n;           // Quiet compiler warning
			if (graph_ < n.graph_ || (graph_ == n.graph_ && index_ < n.index_))
				return true;
            else
				return false;
        }

    private:
        // Allow Graph to access Node's private member data and functions.
        friend class Graph;
        // HW0: YOUR CODE HERE
        // Use this space to declare private data members and methods for Node
        // that will not be visible to users, but may be useful within Graph.
        // i.e. Graph needs a way to construct valid Node objects

		/** Constructs a Node corresponding to given index and graph */
		Node(const Graph& graph, size_type index)
			: index_(index), graph_(&graph){
		}
        // The element's index in the Graph container
        size_type index_;
        // Pointer back to the Graph container
        const Graph *graph_;
    };

    /** Return the number of nodes in the graph.
     *
     * Complexity: O(1).
     */
    size_type size() const {
        // HW0: YOUR CODE HERE
        return points.size();
    }

    /** Synonym for size(). */
    size_type num_nodes() const {
        return size();
    }

    /** Add a node to the graph, returning the added node.
     * @param[in] position The new node's position
     * @post new num_nodes() == old num_nodes() + 1
     * @post result_node.index() == old num_nodes()
     *
     * Complexity: O(1) amortized operations.
     */
    Node add_node(const Point& position) {
        // HW0: YOUR CODE HERE
        (void) position;      // Quiet compiler warning
		points.push_back(position);
        return Node(*this, size()-1);        // Invalid node
    }

    /** Determine if a Node belongs to this Graph
     * @return True if @a n is currently a Node of this Graph
     *
     * Complexity: O(1).
     */
    bool has_node(const Node& n) const {
        // HW0: YOUR CODE HERE
        (void) n;            // Quiet compiler warning
		for (size_type i = 0; i != size(); ++i) {
			if (n == node(i))
				return true;
		}
        return false;
    }

    /** Return the node with index @a i.
     * @pre 0 <= @a i < num_nodes()
     * @post result_node.index() == i
     *
     * Complexity: O(1).
     */
    Node node(size_type i) const {
        // HW0: YOUR CODE HERE
        (void) i;             // Quiet compiler warning
        return Node(*this, i);        // Invalid node
    }

    //
    // EDGES
    //

    /** @class Graph::Edge
     * @brief Class representing the graph's edges.
     *
     * Edges are order-insensitive pairs of nodes. Two Edges with the same nodes
     * are considered equal if they connect the same nodes, in either order.
     */
    class Edge {
    public:
        /** Construct an invalid Edge. */
        Edge() {
            // HW0: YOUR CODE HERE
        }

        /** Return a node of this Edge */
        Node node1() const {
            // HW0: YOUR CODE HERE
            return n1_;      // Invalid Node
        }

        /** Return the other node of this Edge */
        Node node2() const {
            // HW0: YOUR CODE HERE
            return n2_;      // Invalid Node
        }

        /** Test whether this edge and @a e are equal.
         *
         * Equal edges represent the same undirected edge between two nodes.
         */
        bool operator==(const Edge& e) const {
            (void) e;           // Quiet compiler warning
			if (n1_ == e.n1_ && n2_ == e.n2_)
				return true;
			else
				return false;
        }

        /** Test whether this edge is less than @a e in a global order.
         *
         * This ordering function is useful for STL containers such as
         * std::map<>. It need not have any interpretive meaning.
         */
        bool operator<(const Edge& e) const {
            (void) e;           // Quiet compiler warning
			if (n1_ < e.n1_ || (n1_ == e.n1_ && n2_ < e.n2_))
				return true;
			else
				return false;
        }

    private:
        // Allow Graph to access Edge's private member data and functions.
        friend class Graph;
        // HW0: YOUR CODE HERE
        // Use this space to declare private data members and methods for Edge
        // that will not be visible to users, but may be useful within Graph.
        // i.e. Graph needs a way to construct valid Edge objects

		/** Constructs an edge given graph, index, and two nodes*/
		Edge(const Node& n1, const Node& n2)
			: n1_(n1), n2_(n2){
		}

		// Two nodes of the edge
		Node n1_, n2_;
    };

    /** Return the total number of edges in the graph.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    size_type num_edges() const {
        // HW0: YOUR CODE HERE
        return edges.size();
    }

    /** Return the edge with index @a i.
     * @pre 0 <= @a i < num_edges()
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    Edge edge(size_type i) const {
        // HW0: YOUR CODE HERE
        (void) i;             // Quiet compiler warning
        return edges[i];        // Invalid Edge
    }

    /** Test whether two nodes are connected by an edge.
     * @pre @a a and @a b are valid nodes of this graph
     * @return True if for some @a i, edge(@a i) connects @a a and @a b.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    bool has_edge(const Node& a, const Node& b) const {
        // HW0: YOUR CODE HERE
        (void) a; (void) b;   // Quiet compiler warning
		Edge tmpEdge = Edge(a, b);
		for (size_type i = 0; i != num_edges(); ++i) {
			if (tmpEdge == edges[i])
				return true;
		}
        return false;
    }

    /** Add an edge to the graph, or return the current edge if it already exists.
     * @pre @a a and @a b are distinct valid nodes of this graph
     * @return an Edge object e with e.node1() == @a a and e.node2() == @a b
     * @post has_edge(@a a, @a b) == true
     * @post If old has_edge(@a a, @a b), new num_edges() == old num_edges().
     *       Else,                        new num_edges() == old num_edges() + 1.
     *
     * Can invalidate edge indexes -- in other words, old edge(@a i) might not
     * equal new edge(@a i). Must not invalidate outstanding Edge objects.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    Edge add_edge(const Node& a, const Node& b) {
        // HW0: YOUR CODE HERE
        (void) a, (void) b;   // Quiet compiler warning
		edges.push_back(Edge(a, b));
        return edges[num_edges()-1];        // Invalid Edge
    }

    /** Remove all nodes and edges from this graph.
     * @post num_nodes() == 0 && num_edges() == 0
     *
     * Invalidates all outstanding Node and Edge objects.
     */
	void clear() {
		// HW0: YOUR CODE HERE
		*this = Graph();
	}

private:

    // HW0: YOUR CODE HERE
    // Use this space for your Graph class's internals:
    //   helper functions, data members, and so forth.

	std::vector<Point> points;
	std::vector<typename Graph::Edge> edges;

};

#endif // CME212_GRAPH_HPP
