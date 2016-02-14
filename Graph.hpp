#ifndef CME212_GRAPH_HPP
#define CME212_GRAPH_HPP

/** @file Graph.hpp
 * @brief An undirected graph type
 */

#include <algorithm>
#include <vector>
#include <cassert>
#include <unordered_map>

#include "CME212/Util.hpp"
#include "CME212/Point.hpp"

/** A wrapper of int to provide "strongly typed" int. It supports implicit conversion
 * to the underlying int so that it can directly be used as a subscript, but forbids
 * implicit conversion *from* other types.
 */
template <typename TAG>
struct IntWrapper {
	using IntType = unsigned;
	IntWrapper() = default;
    explicit IntWrapper(IntType v_): v(v_) {}
    operator IntType() const {
        return v;
    }
	IntWrapper& operator++() {
		++v;
		return *this;
	}
	IntWrapper& operator--() {
		--v;
		return *this;
	}
	bool operator==(const IntWrapper& rhs) const {
		return v == rhs.v;
	}
	bool operator<(const IntWrapper& rhs) const {
		return v < rhs.v;
	}
	IntType v;
};

namespace std {
	// Hash function for IntWrapper
	template <typename TAG>
	struct hash<IntWrapper<TAG>> {
		typedef IntWrapper<TAG> argument_type;
		typedef size_t result_type;
		result_type operator()(const argument_type& a) const {
			return std::hash<int>()(a.v);
		}
	};
}

/** @class Graph
 * @brief A template for 3D undirected graphs.
 *
 * Users can add and retrieve nodes and edges. Edges are unique (there is at
 * most one edge between any pair of distinct nodes).
 */
template <typename V, typename E = bool>
class Graph {
private:
    // Internal data members are declared at the end
public:

    //
    // PUBLIC TYPE DEFINITIONS
    //

    typedef V node_value_type;
    typedef E edge_value_type;

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
	struct size_type_tag;
	struct uid_type_tag;
    using size_type = IntWrapper<size_type_tag>;
    using uid_type = IntWrapper<uid_type_tag>;

    struct NodeInfo {
        Point p;
        node_value_type v;
        size_type idx;
        bool alive = true;

        NodeInfo(Point p_, node_value_type v_, size_type idx_):
            p(p_), v(v_), idx(idx_) {}
    };


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
    class Node: private totally_ordered<Node> {
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
            return graph_->nodeInfo_[uid_].p;
        }

        Point& position() {
            return graph_->nodeInfo_[uid_].p;
        }

        /** Return this node's index, a number in the range [0, graph_size). */
        size_type index() const {
            return graph_->nodeInfo_[uid_].idx;
        }

        /** Test whether this node and @a n are equal.
         *
         * Equal nodes have the same graph and the same index.
         */
        bool operator==(const Node& n) const {
            return (graph_ == n.graph_ && uid_ == n.uid_);
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
            // Order is defined as dictionary order in (graph_, idx_)
            return (graph_ < n.graph_ || (graph_ == n.graph_ && uid_ < n.uid_));
        }

        /** Return the value of the node */
        node_value_type& value() {
            return graph_->nodeInfo_[uid_].v;
        }

        /** Return the value of the node, const version */
        const node_value_type& value() const {
            return graph_->nodeInfo_[uid_].v;
        }

        /** Return the degree of the node */
        typename size_type::IntType degree() const {
            return graph_->adjList_[uid_].size();
        }

        /** Return the beginning position of incident_iterator */
        incident_iterator edge_begin() const {
            return IncidentIterator(*this, graph_->adjList_[uid_].begin());
        }

        /** Return the end position of incident_iterator */
        incident_iterator edge_end() const {
            return IncidentIterator(*this, graph_->adjList_[uid_].end());
        }

    private:
        friend class Graph;
        friend class incidentIterator;

        /** Constructs a Node corresponding to given index and graph */
        Node(const Graph* graph, uid_type uid)
            : graph_(const_cast<Graph*>(graph)), uid_(uid) {}

        // Pointer back to the Graph container
        Graph *graph_;
        // The element's index in the Graph container
        uid_type uid_;
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
    class Edge: private totally_ordered<Edge> {
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
            // For the same graph, order is defined as dictionary order in (min(i1_,
            // i2_), max(i1_, i2_))
            auto m = std::min(i1_, i2_), me = std::min(e.i1_, e.i2_);
            return (graph_ < e.graph_)
				|| (graph_ == e.graph_
					&&  (m < me || (m == me && std::max(i1_, i2_) < std::max(e.i1_, e.i2_))));
        }

        /** Return the length of the edge */
        double length() {
            return norm(graph_->nodeInfo_[i1_].p - graph_->nodeInfo_[i2_].p);
        }


        /** Return this edge's value */
        edge_value_type& value() {
            // Since the edgeIterator only iterates over edges with smaller first
            // element, the value is only stored for these edges; otherwise, an
            // initialization by edgeIterator is not able to update all edge values
            // correctly.
            uid_type m = std::min(i1_, i2_);
            uid_type M = std::max(i1_, i2_);
            auto pos = graph_->adjList_[m].find(M);
            return pos->second;
        }

        const edge_value_type& value() const {
            uid_type m = std::min(i1_, i2_);
            uid_type M = std::max(i1_, i2_);
            auto pos = graph_->adjList_[m].find(M);
            return pos->second;
        }

    private:
        // Allow Graph to access Edge's private member data and functions.
        friend class Graph;

        // Constructs an edge given a graph pointer and two node uids
        explicit Edge(const Graph* graph, const uid_type& i1, const uid_type& i2)
            : i1_(i1), i2_(i2), graph_(const_cast<Graph*>(graph)) {}

        // Uids of the two endpoints of the edge
        uid_type i1_, i2_;
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
            return graph_->node(idx_);
        }

        /** Advance to next position and return the new position*/
        node_iterator& operator++() {
            ++idx_;
            return *this;
        }

        /** Test whether two iterators are at the same position */
        bool operator==(const node_iterator& rhs) const {
            return idx_ == rhs.idx_ && graph_ == rhs.graph_;
        }

    private:
        friend class Graph;

        /** Construct a NodeIterator with the index of the underlying Node */
        NodeIterator(const Graph* graph, size_type idx): idx_(idx), graph_(const_cast<Graph*>(graph)) {}

        // The index of Node it is pointing to
        size_type idx_;
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
            return Edge(graph_, i1_, i2_->first);
        }

        /** Avanced to next position */
        edge_iterator& operator++() {
            ++i2_;
            fix();
            return *this;
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
        uid_type i1_;
        // Pointer back to the Graph contrainer
        Graph *graph_;
        // The set iterator at the other endpoint
        typename std::unordered_map<uid_type, E>::iterator i2_;


        // Fix the internal data when they do not satisfy the representation
        // invariant; to avoid duplication, we skip those with smaller second index
        void fix() {
            while (i1_ < graph_->adjList_.size()) {
                while (i2_ != graph_->adjList_[i1_].end()) {
                    while (i1_ < i2_->first)
                        return;
                    ++i2_;
                }
                ++i1_;
                i2_ = graph_->adjList_[i1_].begin();
            }
        }

        // Construct an EdgeIterator with one node, starting at the beginning of that
        // node's adjacency list
        EdgeIterator(const Graph* graph, uid_type i1): i1_(i1), graph_(const_cast<Graph*>(graph)) {
            if (i1 < graph_->adjList_.size()) i2_ = graph_->adjList_[i1_].begin();
            fix();
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
            return Edge(n_.graph_, n_.uid_, pos_->first);
        }

        /** Return the other endpoint of the edge */
        Node node2() const {
            return n_.graph_->node(n_.graph_->uid2idx(pos_->first));
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
        typename std::unordered_map<uid_type, E>::iterator pos_;
        // Construct an IncidentIterator with Node index and an iterator at that
        // Node's adjacency list
        IncidentIterator(const Node& n, typename std::unordered_map<uid_type, E>::iterator pos)
            : n_(n), pos_(pos) {}
    };


    /** Return the number of nodes in the graph.
     *
     * Complexity: O(1).
     */
    size_type size() const {
        return nNodes_;
    }

    /** Synonym for size(). */
    size_type num_nodes() const {
        return size();
    }

    /** Translate uid to index */
    size_type uid2idx(uid_type uid) const {
        return nodeInfo_[uid].idx;
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
        if (!removedUids_.size()) {
            nodeInfo_.emplace_back(position, nValue, nNodes_);
            idx2uid_.emplace_back(nNodes_);
            ++nNodes_;
            adjList_.push_back(std::unordered_map<uid_type, E>());
            return Node(this, uid_type(nNodes_ - 1));
        }
        else {
            uid_type reuse = removedUids_.back();
            removedUids_.erase(removedUids_.end()-1);
            nodeInfo_[reuse].p = position;
            nodeInfo_[reuse].v = nValue;
            nodeInfo_[reuse].idx = nNodes_;
            nodeInfo_[reuse].alive = true;
            idx2uid_[nNodes_] = reuse;
            ++nNodes_;
            return Node(this, reuse);
        }
    }

    /** Remove a node from the graph.
     * @param[in] n The node to be removed
     * @pre has_node(@a n) == true
     * @post Node @a n along with all its edges are removed from the graph
     * @post new num_nodes() == old num_nodes() - 1
     * @return The index of the removed node (before removal)
     *
     * Amortized time complexity is O(@a n.degree())
     * The indices of the remaining nodes may be changed.
     * All existing node_iterators are invalidated, and
     * all Node objects corresponding to @a n are invalidated
     */
    size_type remove_node(const Node& n) {
        assert(has_node(n));
        removedUids_.push_back(n.uid_);
        nodeInfo_[n.uid_].alive = false;

        // Remove all edges associated to this node
        for (auto j = adjList_[n.uid_].begin(); j != adjList_[n.uid_].end(); ++j) {
            adjList_[j->first].erase(n.uid_);
            --nEdges_;
        }
        adjList_[n.uid_].clear();

        // Reindex nodes
        idx2uid_[n.index()] = idx2uid_[nNodes_-1];
        nodeInfo_[idx2uid_[n.index()]].idx = n.index();
        --nNodes_;

        return n.index();
    }

    /** Remove a node from the graph
     * @param[in] n_it A node_iterator at the node to be removed
     * @pre The underlying node belongs to the graph
     * @post The node associated to @a n_it along with all its edges are removed
     * from the graph
     * @post new num_nodes() == old num_nodes() -1
     * @return A node_iterator at the node that occupies the removed node's index
     * after the removal
     *
     * Amortized time complexity is O(degree of the removed node)
     * The indices of the remaining nodes may be changed.
     * All existing node_iterators are invalidated, and
	 * all Node objects corresponding to @a n are invalidated
     */
    node_iterator remove_node(node_iterator n_it) {
        return NodeIterator(this, remove_node(*n_it));
    }

    /** Remove an edge from the graph
     * @param[in] n1 one node of the edge
     * @param[in] n2 the other node of the edge
     * @pre @a n1 and @a n2 both belong to this graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(1)
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(const Node& n1, const Node& n2) {
        if (has_edge(n1, n2)) {
            adjList_[n1.uid_].erase(n2.uid_);
            adjList_[n2.uid_].erase(n1.uid_);
            --nEdges_;
            return true;
        }
        else
            return false;
    }

    /** Remove an edge from the graph
     * @param[in] e the edge to be removed
     * @pre @a e belongs to the graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(1)
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(const Edge& e) {
        Node n1 = e.node1();
        Node n2 = e.node2();
        return remove_edge(n1, n2);
    }

    /** Remove an edge from the graph
     * @param[in] e_it an edge_iterator at the edge to be removed
     * @pre The underlying edge belongs to the graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(1)
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(edge_iterator e_it) {
        Edge e = *e_it;
        return remove_edge(e);
    }


    /** Determine if a Node belongs to this Graph
     * @return True if @a n is currently a Node of this Graph
     *
     * Complexity: O(1).
     */
    bool has_node(const Node& n) const {
        return n.graph_ == this
            && nodeInfo_[n.uid_].alive;
    }

    /** Return the node with index @a i.
     * @pre 0 <= @a i < num_nodes()
     * @post result_node.index() == i
     *
     * Complexity: O(1).
     */
    Node node(size_type i) const {
        return Node(this, idx2uid_[i]);
    }
	
	Node node(typename size_type::IntType i) const {
		return Node(this, idx2uid_[i]);
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
    Edge edge(size_type i) const {
        return *std::next(edge_begin(), i);
    }
	
	Edge edge(typename size_type::IntType i) const {
        return *std::next(edge_begin(), i);
    }
	
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
        return (a.graph_ == b.graph_)
            && (a.graph_ == this)
            && adjList_[a.uid_].find(b.uid_) != adjList_[a.uid_].end();
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
            adjList_[a.uid_].insert({ b.uid_, E() });
            adjList_[b.uid_].insert({ a.uid_, E() });
        }
        return Edge(this, a.uid_, b.uid_);
    }

    /** Remove all nodes and edges from this graph.
     * @post num_nodes() == 0 && num_edges() == 0
     *
     * Invalidates all outstanding Node and Edge objects.
     */
    void clear() {
        nEdges_.v = 0;
        nNodes_.v = 0;
        nodeInfo_.clear();
        adjList_.clear();
        idx2uid_.clear();
        removedUids_.clear();
    }

    /** Return an iterator pointing to the first node */
    node_iterator node_begin() const {
        return node_iterator(this, size_type(0));
    }
    /** Return an iterator pointing to the next position of the last node */
    node_iterator node_end() const {
        return node_iterator(this, nNodes_);
    }
    /** Return an iterator pointing to the first edge */
    edge_iterator edge_begin() const {
        return edge_iterator(this, uid_type(0));
    }
    /** Return an iterator pointing to the next position of the last edge */
    edge_iterator edge_end() const {
        return edge_iterator(this, uid_type(adjList_.size()));
    }

private:
    size_type nEdges_{0};
    size_type nNodes_{0};
    // EdgesType edges_; [[deprecated]]
    // Indexed by uid
    std::vector<NodeInfo> nodeInfo_;
    // Adjacency list/set; each node has a set that consists of its adjacent nodes.
    // Indexed by uid
    std::vector<std::unordered_map<uid_type, E>> adjList_;
    std::vector<uid_type> idx2uid_;
    std::vector<uid_type> removedUids_;
};


template <typename Graph>
struct NodesRange {
    typename Graph::node_iterator begin() {
        return g.node_begin();
    }
    typename Graph::node_iterator end() {
        return g.node_end();
    }
    Graph& g;
};


template <typename Graph>
NodesRange<Graph> nodesRange(Graph& g) {
    return {g};
}

template <typename Graph>
struct EdgesRange {
    typename Graph::edge_iterator begin() {
        return g.edge_begin();
    }
    typename Graph::edge_iterator end() {
        return g.edge_end();
    }
    Graph& g;
};


template <typename Graph>
EdgesRange<Graph> edgesRange(Graph& g) {
    return {g};
}


#endif // CME212_GRAPH_HPP
