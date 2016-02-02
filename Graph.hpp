#ifndef CME212_GRAPH_HPP
#define CME212_GRAPH_HPP

/** @file Graph.hpp
 * @brief An undirected graph type
 */

#include <algorithm>
#include <vector>
#include <cassert>
#include <unordered_set>

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
    using AdjListType = std::vector<std::unordered_set<size_type>>;
    using ValuesType = std::vector<node_value_type>;

    //
    // CONSTRUCTORS AND DESTRUCTOR
    //

    /** Construct an empty graph. */
    Graph() {}

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
        Node() {};

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
            return (graph_ < n.graph_ || (graph_ == n.graph_ && index_ < n.index_));
        }

        /** Return the value of the node */
        node_value_type& value() {
            return graph_->nValues_[index_];
        }

        /** Return the value of the node, const version */
        const node_value_type& value() const {
            return graph_->nValues_[index_];
        }

        /** Return the degree of the node */
        size_type degree() const {
            return graph_->adjList_[index_].size();
        }

        /** Return the beginning position of incident_iterator */
        incident_iterator edge_begin() const {
            return IncidentIterator(*this, graph_->adjList_[index_].begin());
        }

        /** Return the end position of incident_iterator */
        incident_iterator edge_end() const {
            return IncidentIterator(*this, graph_->adjList_[index_].end());
        }

    private:
        // Allow Graph to access Node's private member data and functions.
        friend class Graph;

        /** Constructs a Node corresponding to given index and graph */
        Node(const Graph* graph, size_type index)
            : index_(index), graph_(const_cast<Graph*>(graph)) {}

        // The element's index in the Graph container
        size_type index_;
        // Pointer back to the Graph container
        Graph *graph_;
    };


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
        Edge() {};

        /** Return a node of this Edge */
        Node node1() const {
            return Node(graph_, i1_);
        }

        /** Return the other node of this Edge */
        Node node2() const {
            return Node(graph_, i2_);
        }

        /** Test whether this edge and @a e are equal.
         *
         * Equal edges represent the same undirected edge between two nodes.
         */
        bool operator==(const Edge& e) const {
            return ((i1_ == e.i1_ && i2_ == e.i2_)
                    ||  (i1_ == e.i2_ && i2_ == e.i1_))
                && graph_ == e.graph_;
        }

        /** Test whether this edge is less than @a e in a global order.
         *
         * This ordering function is useful for STL containers such as
         * std::map<>. It need not have any interpretive meaning.
         */
        bool operator<(const Edge& e) const {
            // Order is defined as dictionary order in (min(i1_, i2_), max(i1_, i2_))
            auto m = std::min(i1_, i2_), me = std::min(e.i1_, e.i2_);
            return (m < me || (m == me && std::max(i1_, i2_) < std::max(e.i1_, e.i2_)));
        }

    private:
        // Allow Graph to access Edge's private member data and functions.
        friend class Graph;

        /** Constructs an edge given a graph pointer and two node indices*/
        Edge(const Graph* graph, const size_type& i1, const size_type& i2)
            : i1_(i1), i2_(i2), graph_(const_cast<Graph*>(graph)) {}

        // Indices of the two endpoints of the edge
        size_type i1_, i2_;
        // Pointer back to the Graph contrainer
        Graph *graph_;
    };


    //
    // Node Iterator
    //

    /** @class Graph::NodeIterator
     * @brief Iterator class for nodes. A forward iterator. */
    class NodeIterator: private totally_ordered<NodeIterator> {
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
        NodeIterator() {};

        /** Deference the iterator */
        Node operator*() const {
            return graph_->node(id_);
        }

        /** Advance to next position and return the new position*/
        node_iterator& operator++() {
            ++id_;
            return *this;
        }

        /** Test whether two iterators are at the same position */
        bool operator==(const node_iterator& rhs) const {
            return id_ == rhs.id_ && graph_ == rhs.graph_;
        }

    private:
        friend class Graph;

        /** Construct a NodeIterator with the index of the underlying Node */
        NodeIterator(const Graph* graph, size_type id): id_(id), graph_(const_cast<Graph*>(graph)) {}

        // The index of Node it is pointing to
        size_type id_;
        // Pointer back to the Graph contrainer
        Graph *graph_;
    };


    //
    // Edge Iterator
    //

    /** @class Graph::EdgeIterator
     * @brief Iterator class for edges. A forward iterator. */
    class EdgeIterator: private totally_ordered<EdgeIterator> {
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
        EdgeIterator() {};

        /** Deference the iterator */
        Edge operator*() const {
            return Edge(graph_, i1_, *i2_);
        }

        /** Avanced to next position */
        edge_iterator& operator++() {
			++i2_;
			fix();
            return *this;
        }

		/** Fix the internal data when they do not satisfy the representation
		 * invariant; to avoid duplication, we skip those with smaller second index
		 */
		void fix() {
			while (i1_ < graph_->adjList_.size()) {
				while (i2_ != graph_->adjList_[i1_].end()) {
					while (i1_ < *i2_)
						return;
					++i2_;
				}
				++i1_;
				i2_ = graph_->adjList_[i1_].begin();
			}
		}

        /** Test whether two EdgeIterators have the same position */
        bool operator==(const edge_iterator& eit) const {
            return i1_ == eit.i1_
                && (i1_ == graph_->adjList_.size()
                    || i2_ == eit.i2_)
                && graph_ == eit.graph_;
        }

    private:
        friend class Graph;
        // The index of the smaller endpoint
        size_type i1_;
        // Pointer back to the Graph contrainer
        Graph *graph_;
        // The set iterator at the other endpoint
        typename std::unordered_set<size_type>::iterator i2_;

        // Construct an EdgeIterator with one node, starting at the beginning of that
        // node's adjacency list
        EdgeIterator(const Graph* graph, size_type i1): i1_(i1), graph_(const_cast<Graph*>(graph)) {
            if (i1 < graph_->adjList_.size()) i2_ = graph_->adjList_[i1_].begin();
        }
    };


    //
    // Incident Iterator
    //

    /** @class Graph::IncidentIterator
     * @brief Iterator class for edges incident to a node. A forward iterator. */
    class IncidentIterator: private totally_ordered<IncidentIterator> {
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
        IncidentIterator() {}

        /** Deference the iterator */
        Edge operator*() const {
            return Edge(n_.graph_, n_.index(), *pos_);
        }

        /** Return the other endpoint of the edge */
        Node node2() const {
            return *pos_;
        }

        /** Advanced to next position */
        incident_iterator& operator++() {
            ++pos_;
            return *this;
        }

        /** Test whether two iterators are pointing to the same position */
        bool operator==(const incident_iterator& iit) const {
            return (pos_ == iit.pos_)
                && (n_ == iit.n_);
        }

    private:
        friend class Node;

        // The underlying Node
        Node n_;
        // Iterator at adjList_
        std::unordered_set<size_type>::iterator pos_;
        // Construct an incidentIterator with Node index and an iterator at that
        // Node's adjacency list
        IncidentIterator(const Node& n, std::unordered_set<size_type>::iterator pos)
            : n_(n), pos_(pos) {}
    };


    /** Return the number of nodes in the graph.
     *
     * Complexity: O(1).
     */
    size_type size() const {
        return points_.size();
    }

    /** Synonym for size(). */
    size_type num_nodes() const {
        return size();
    }

    /** Add a node to the graph, returning the added node.
     * @param[in] position The new node's position
     * @param[in] nValue The new node's value
     * @post new num_nodes() == old num_nodes() + 1
     * @post result_node.index() == old num_nodes()
     *
     * Complexity: O(1) amortized operations.
     */
    Node add_node(const Point& position, const node_value_type& nValue = node_value_type()) {
        points_.push_back(position);
        nValues_.push_back(nValue);
        adjList_.push_back(std::unordered_set<size_type>());
        return Node(this, size()-1);
    }

    /** Determine if a Node belongs to this Graph
     * @return True if @a n is currently a Node of this Graph
     *
     * Complexity: O(1).
     */
    bool has_node(const Node& n) const {
        return (n.graph_ == this);
    }

    /** Return the node with index @a i.
     * @pre 0 <= @a i < num_nodes()
     * @post result_node.index() == i
     *
     * Complexity: O(1).
     */
    Node node(size_type i) const {
        return Node(this, i);
    }

    /** Return the total number of edges in the graph.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    size_type num_edges() const {
        return nEdges_;
    }

    // [[deprecated]]
    // size_type num_edges() const {
    //     return edges_.size();
    // }

    /** Return the edge with index @a i.
     * @pre 0 <= @a i < num_edges()
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    // [[deprecated]]
    // Edge edge(size_type i) const {
    //  return edges_[i];
    // }

    /** Test whether two nodes are connected by an edge.
     * @pre @a a and @a b are valid nodes of this graph
     * @return True if for some @a i, edge(@a i) connects @a a and @a b.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    bool has_edge(const Node& a, const Node& b) const {
        size_type m = std::min(a.index(), b.index());
        size_type M = std::max(a.index(), b.index());
        return (a.graph_ == b.graph_)
            && (a.graph_ == this)
            && adjList_[m].find(M) != adjList_[m].end();
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
        if (!has_edge(a, b)) {
            ++nEdges_;
            adjList_[a.index()].insert(b.index());
            adjList_[b.index()].insert(a.index());
        }
        return Edge(this, a.index(), b.index());
    }

    /** Remove all nodes and edges from this graph.
     * @post num_nodes() == 0 && num_edges() == 0
     *
     * Invalidates all outstanding Node and Edge objects.
     */
    void clear() {
        nEdges_ = 0;
        points_.clear();
        adjList_.clear();
        nValues_.clear();
    }

    /** Return an iterator pointing to the first node */
    node_iterator node_begin() const {
        return node_iterator(this, 0);
    }
    /** Return an iterator pointing to the next position of the last node */
    node_iterator node_end() const {
        return node_iterator(this, points_.size());
    }
    /** Return an iterator pointing to the first edge */
    edge_iterator edge_begin() const {
        return edge_iterator(this, 0);
    }
    /** Return an iterator pointing to the next position of the last edge */
    edge_iterator edge_end() const {
        return edge_iterator(this, adjList_.size());
    }

private:
    PointsType points_;
    size_type nEdges_ = 0;
    // EdgesType edges_; [[deprecated]]
    /** Adjacency list/set; each node has a set that consists of its adjacent nodes. */
    AdjListType adjList_;
    ValuesType nValues_;
};

#endif // CME212_GRAPH_HPP
