#ifndef CME212_GRAPH_HPP
#define CME212_GRAPH_HPP

/** @file Graph.hpp
 * @brief An undirected graph type
 */

#include <algorithm>
#include <vector>
#include <cassert>
#include <set>

#include "CME212/Util.hpp"
#include "CME212/Point.hpp"


/** @class Graph
 * @brief A template for 3D undirected graphs.
 *
 * Users can add and retrieve nodes and edges. Edges are unique (there is at
 * most one edge between any pair of distinct nodes).
 */
template <typename V>
class Graph {
private:
    // Internal data members are declared at the end
public:

    //
    // PUBLIC TYPE DEFINITIONS
    //

	typedef V node_value_type;

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

    /** Type of node iterators, which iterate over all graph nodes. */
    class NodeIterator;
    /** Synonym for NodeIterator */
    typedef NodeIterator node_iterator;

    /** Type of edge iterators, which iterate over all graph edges. */
    class EdgeIterator;
    /** Synonym for EdgeIterator */
    typedef EdgeIterator edge_iterator;

    /** Type of incident iterators, which iterate incident edges to a node. */
    class IncidentIterator;
    /** Synonym for IncidentIterator */
    typedef IncidentIterator incident_iterator;

    /** Type of indexes and sizes.
        Return type of Graph::Node::index(), Graph::num_nodes(),
        Graph::num_edges(), and argument type of Graph::node(size_type)
    */
    typedef unsigned size_type;

	using PointsType = std::vector<Point>;
	using EdgesType = std::vector<Edge>;
	using AdjListType = std::vector<std::set<size_type>>;
	using ValuesType = std::vector<node_value_type>;

    //
    // CONSTRUCTORS AND DESTRUCTOR
    //

    /** Construct an empty graph. */
    Graph(): points_({}), edges_({}) {}

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
        Node() {}

        /** Return this node's position. */
        const Point& position() const {
            return graph_->points_[index_];
        }

        /** Return this node's index, a number in the range [0, graph_size). */
        size_type index() const {
            return index_;
        }

        /** Test whether this node and @a n are equal.
         *
         * Equal nodes have the same graph and the same index.
         */
        bool operator==(const Node& n) const {
            return (graph_ == n.graph_ && index_ == n.index_);
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
            // Order is defined as dictionary order in (graph_, index_)
            return (graph_ < n.graph_ || (graph_ == n.graph_ && index_ < n.index_))
        }

		/** Return the value of the node */
		node_value_type& value() {
			return graph_->nValues_[index_];
		}

		/** Return the value of the node, const version */
		const node_value_type& value() const {
			return graph_->nValues_[index_];
		}

    private:
        // Allow Graph to access Node's private member data and functions.
        friend class Graph;

        /** Constructs a Node corresponding to given index and graph */
        Node(const Graph* graph, size_type index)
            : index_(index), graph_(const_cast<Graph*>(graph)) {

        }
        // The element's index in the Graph container
        size_type index_;
        // Pointer back to the Graph container
        Graph *graph_;
    };

    /** Return the number of nodes in the graph.
     *
     * Complexity: O(1).
     */
    size_type size() const;
    /** Synonym for size(). */
    size_type num_nodes() const;
    /** Add a node to the graph, returning the added node.
     * @param[in] position The new node's position
     * @param[in] nValue The new node's value
     * @post new num_nodes() == old num_nodes() + 1
     * @post result_node.index() == old num_nodes()
     *
     * Complexity: O(1) amortized operations.
     */
    Node add_node(const Point& position, const node_value_type& nValue = node_value_type());
    /** Determine if a Node belongs to this Graph
     * @return True if @a n is currently a Node of this Graph
     *
     * Complexity: O(1).
     */
    bool has_node(const Node& n) const;
    /** Return the node with index @a i.
     * @pre 0 <= @a i < num_nodes()
     * @post result_node.index() == i
     *
     * Complexity: O(1).
     */
    Node node(size_type i) const;
    // HW1: YOUR CODE HERE
    // Supply definitions AND SPECIFICATIONS for:
    // node_value_type& value();
    // const node_value_type& value() const;
    // size_type degree() const;
    // incident_iterator edge_begin() const;
    // incident_iterator edge_end() const;

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
        Edge();
        /** Return a node of this Edge */
        Node node1() const;
        /** Return the other node of this Edge */
        Node node2() const;
        /** Test whether this edge and @a e are equal.
         *
         * Equal edges represent the same undirected edge between two nodes.
         */
        bool operator==(const Edge& e) const;
        /** Test whether this edge is less than @a e in a global order.
         *
         * This ordering function is useful for STL containers such as
         * std::map<>. It need not have any interpretive meaning.
         */
        bool operator<(const Edge& e) const;

    private:
        // Allow Graph to access Edge's private member data and functions.
        friend class Graph;

        /** Constructs an edge given a graph pointer and two node indices*/
        Edge(const Graph* graph, const size_type& i1, const size_type& i2);

        // Two nodes of the edge
        size_type i1_, i2_;
        // Pointer back to the Graph contrainer
        Graph *graph_;
    };

    //
    // Node Iterator
    //

    /** @class Graph::NodeIterator
     * @brief Iterator class for nodes. A forward iterator. */
    class NodeIterator {
    public:
        // These type definitions help us use STL's iterator_traits.
        /** Element type. */
        typedef Node value_type;
        /** Type of pointers to elements. */
        typedef Node* pointer;
        /** Type of references to elements. */
        typedef Node& reference;
        /** Iterator category. */
        typedef std::input_iterator_tag iterator_category;
        /** Difference between iterators */
        typedef std::ptrdiff_t difference_type;

        /** Construct an invalid NodeIterator. */
        NodeIterator();
		/** Construct a NodeIterator with the index of the underlying Node */
		NodeIterator(size_type id_);

		/** Deference the iterator */
		Node operator*() const;
		/** Advance to next position and return the new position*/
		node_iterator& operator++();
		/** Test whether two iterators are at the same position */
		bool operator==(const node_iterator& rhs) const;

    private:
        friend class Graph;

		size_type id_;
    };


    //
    // Edge Iterator
    //

    /** @class Graph::EdgeIterator
     * @brief Iterator class for edges. A forward iterator. */
    class EdgeIterator {
    public:
        // These type definitions help us use STL's iterator_traits.
        /** Element type. */
        typedef Edge value_type;
        /** Type of pointers to elements. */
        typedef Edge* pointer;
        /** Type of references to elements. */
        typedef Edge& reference;
        /** Iterator category. */
        typedef std::input_iterator_tag iterator_category;
        /** Difference between iterators */
        typedef std::ptrdiff_t difference_type;

        /** Construct an invalid EdgeIterator. */
        EdgeIterator();
		/** Deference the iterator */
		Edge operator*() const;
		/** Avanced to next position */
		edge_iterator& operator++();
		/** Test whether two EdgeIterators have the same position */
		bool operator==(const edge_iterator& eit) const;

    private:
        friend class Graph;
		// The index of the smaller endpoint
		size_type i1_;
		// The set iterator at the other endpoint
		typename std::set<size_type>::iterator i2_;
		// Construct an EdgeIterator with one node index and one set iterator
		EdgeIterator(size_type i1);
    };


    //
    // Incident Iterator
    //

    /** @class Graph::IncidentIterator
     * @brief Iterator class for edges incident to a node. A forward iterator. */
    class IncidentIterator {
    public:
        // These type definitions help us use STL's iterator_traits.
        /** Element type. */
        typedef Edge value_type;
        /** Type of pointers to elements. */
        typedef Edge* pointer;
        /** Type of references to elements. */
        typedef Edge& reference;
        /** Iterator category. */
        typedef std::input_iterator_tag iterator_category;
        /** Difference between iterators */
        typedef std::ptrdiff_t difference_type;

        /** Construct an invalid IncidentIterator. */
        IncidentIterator() {
        }

        // HW1 #5: YOUR CODE HERE
        // Supply definitions AND SPECIFICATIONS for:
        // Edge operator*() const
        // IncidentIterator& operator++()
        // bool operator==(const IncidentIterator&) const

    private:
        friend class Graph;
        // HW1 #5: YOUR CODE HERE
    };

    /** Return the total number of edges in the graph.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    size_type num_edges() const;

    /** Return the edge with index @a i.
     * @pre 0 <= @a i < num_edges()
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    Edge edge(size_type i) const;

    /** Test whether two nodes are connected by an edge.
     * @pre @a a and @a b are valid nodes of this graph
     * @return True if for some @a i, edge(@a i) connects @a a and @a b.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    bool has_edge(const Node& a, const Node& b) const;

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
    Edge add_edge(const Node& a, const Node& b);
    }

    /** Remove all nodes and edges from this graph.
     * @post num_nodes() == 0 && num_edges() == 0
     *
     * Invalidates all outstanding Node and Edge objects.
     */
    void clear();

	/** Return an iterator pointing to the first node */
	node_iterator node_begin() const;
	/** Return an iterator pointing to the next position of the last node */
	node_iterator node_end() const;
	/** Return an iterator pointing to the first edge */
	edge_iterator edge_begin() const;
	/** Return an iterator pointing to the next position of the last edge */
	edge_iterator edge_end() const;

private:
    PointsType points_;
    EdgesType edges_;
	/** Adjacency list/set; each node has a set that consists of its adjacent nodes. */
	// It is named "List" just in case the internal data structure may be changed later.
	AdjListType adjList_;
	ValuesType nValues_;
};


////////////////////////////////////
// Implementions of Graph methods //
////////////////////////////////////

template <typename V>
size_type Graph<V>::size() const {
	return points_.size();
}

template <typename V>
size_type Graph<V>::num_nodes() const {
	return size();
}

template <typename V>
Node Graph<V>::add_node(const Point& position, const node_value_type& nValue = node_value_type()) {
	points_.push_back(position);
	nValues_.push_back(nValue);
	return Node(this, size()-1);
}

template <typename V>
bool Graph<V>::has_node(const Node& n) const {
	return (n.graph_ == this)
}

template <typename V>
Graph<V>::Node node(size_type i) const {
	return Node(this, i);
}

template <typename V>
Edge Graph<V>::edge(size_type i) const {
	return edges_[i];
}

template <typename V>
size_type Graph<V>::num_edges() const {
	return edges_.size();
}

template <typename V>
bool Graph<V>::has_edge(const Node& a, const Node& b) const {
	if (a.index() > b.index()) std::swap(a, b);
	return (a.graph_ == b.graph_)
		&& (a.graph_ == this)
		&& adjList_[a.index()].find(b.index()) != adjList_[a.index()].end();
}

template <typename V>
Edge Graph<V>::add_edge(const Node& a, const Node& b) {
	if (has_edge(a, b))
		return Edge(this, a.index(), b.index());
	else {
		edges_.push_back(Edge(this, a.index(), b.index()));
		if (a.index() > b.index()) std::swap(a, b);
		adjList_[a.index()].insert(b.index());
		return edges_[num_edges()-1];
	}
}

template <typename V>
void Graph<V>::clear() {
	points_.clear();
	edges_.clear();
	adjList_.clear();
}

template <typename V>
node_iterator Graph<V>::node_begin() const {
	return node_iterator(0);
}

template <typename V>
node_iterator Graph<V>::node_end() const {
	return node_iterator(points_.size());
}

template <typename V>
edge_iterator Graph<V>::edge_begin() const {
	return edge_iterator(0);
}

template <typename V>
edge_iterator Graph<V>::edge_end() const {
	return edge_iterator(adjList_.size());
}


/////////////////////////////////////
// Implementations of Edge methods //
/////////////////////////////////////

template <typename V>
Graph<V>::Edge::Edge() {
}

template <typename V>
Graph<V>::Edge::Edge(const Graph* graph, const size_type& i1, const size_type& i2)
	: i1_(i1), i2_(i2), graph_(const_cast<Graph*>(graph)) {
}

template <typename V>
Node Graph<V>::Edge::node1() const {
	return Node(graph_, i1_);
}

template <typename V>
Node Graph<V>::Edge::node2() const {
	return Node(graph_, i2_);
}

template <typename V>
bool Graph<V>::Edge::operator==(const Edge& e) const {
	return ((i1_ == e.i1_ && i2_ == e.i2_)
			||  (i1_ == e.i2_ && i2_ == e.i1_))
		&& graph_ == e.graph_;
}

template <typename V>
bool Graph<V>::Edge::operator<(const Edge& e) const {
	// Order is defined as dictionary order in (min(i1_, i2_), max(i1_, i2_))
	auto m = std::min(i1_, i2_), me = std::min(e.i1_, e.i2_);
	return (m < me || (m == me && std::max(i1_, i2_) < std::max(e.i1_, e.i2_)));
}


/////////////////////////////////////////////
// Implementations of NodeIterator methods //
/////////////////////////////////////////////

template <typename V>
Graph<V>::NodeIterator::NodeIterator() {}

template <typename V>
Graph<V>::NodeIterator::NodeIterator(size_type id): id_(id) {}

template <typename V>
Node Graph<V>::NodeIterator::operator*() const {
	return node(id_);
}

template <typename V>
node_iterator& Graph<V>::NodeIterator::operator++() {
	++id_;
	return *this;
}

template <typename V>
bool Graph<V>::NodeIterator::operator==(const node_iterator& rhs) const {
	return id_ == rhs.id_;
}


///////////////////////////////////////////
// Implementions of EdgeIterator methods //
///////////////////////////////////////////

template <typename V>
Graph<V>::EdgeIterator::EdgeIterator() {}

template <typename V>
Graph<V>::EdgeIterator::EdgeIterator(size_type i1): i1_(i1) {
	if (i1 < adjList_.size()) i2_ = adjList_[i1_].begin();
}

template <typename V>
Edge Graph<V>::EdgeIterator::operator*() const {
	return Edge(i1_, *i2_);
}

template <typename V>
edge_iterator& Graph<V>::EdgeIterator::operator++() {
	++i2_;
	if (i2_ == adjList_[i1].end()) {
		++i1_;
		if (i1_ < adjList_.size())
			i2_ = adjList_[i1_].begin();
	}
}

template <typename V>
bool Graph<V>::EdgeIterator::operator==(const edge_iterator& eit) {
	return (i1_ == adjList_.size() && eit.i1_ == adjList.size())
		|| (i1_ == it.i1_ && i2_ == it.i2_);
}

#endif // CME212_GRAPH_HPP
