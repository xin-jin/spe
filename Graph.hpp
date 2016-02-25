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
#include "struct/ObjPool.hpp"
#include "struct/IntWrapper.hpp"


/** @class Graph
 * @brief A template for 3D undirected graphs.
 *
 * Users can add and retrieve nodes and edges. Edges are unique (there is at
 * most one edge between any pair of distinct nodes).
 */
template <typename V, typename E = bool>
class Graph {
public:
    //
    // PUBLIC TYPE DEFINITIONS
    //

    struct EdgeInfo;
    struct NodeInfo;

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

    using NodePoolType = ObjPool<NodeInfo>;
    using EdgePoolType = ObjPool<EdgeInfo>;

    /** Type of indexes and sizes.
        Return type of Graph::Node::index(), Graph::num_nodes(),
        Graph::num_edges(), and argument type of Graph::node(size_type)
    */
    struct size_type_tag;
    using size_type = IntWrapper<size_type_tag>;
    using n_uid_type = typename NodePoolType::uid_type;
    using e_uid_type = typename EdgePoolType::uid_type;

    struct EdgeInfo {
        edge_value_type v;
        n_uid_type n1, n2;

        EdgeInfo(n_uid_type n1_, n_uid_type n2_): n1(n1_), n2(n2_) {}

        EdgeInfo(edge_value_type v_, n_uid_type n1_, n_uid_type n2_):
            EdgeInfo(n1_, n2_), v(v_) {}

        EdgeInfo(EdgeInfo&&) = default;

        EdgeInfo& operator=(EdgeInfo&&) = default;
	};

    struct NodeInfo {
        using AdjListType = std::vector<std::pair<n_uid_type, e_uid_type>>;

        Point p;
        node_value_type v;
        size_type idx;
        AdjListType adjList;

        NodeInfo(Point p_, node_value_type v_, size_type idx_):
            p(p_), v(v_), idx(idx_) {}

        NodeInfo(NodeInfo&& ni) = default;

        NodeInfo& operator=(NodeInfo&& n) = default;
	};

private:
    using AdjListType = typename NodeInfo::AdjListType;
    using AdjListItType = typename AdjListType::iterator;
    AdjListItType find_node(AdjListType& adj, n_uid_type n_uid) {
        for (auto i = adj.begin(); i != adj.end(); ++i) {
            if (i->first == n_uid)
                return i;
        }
        return adj.end();
    }

    auto find_node(const AdjListType& adj, n_uid_type n_uid) const -> decltype(adj.begin()) {
        for (auto i = adj.begin(); i != adj.end(); ++i) {
            if (i->first == n_uid)
                return i;
        }
        return adj.end();
    }

    // Given an adjacency list and a node uid, remove the element corresponding
    // to that uid from the list
    // Time complexity is O(the size of the list)
    e_uid_type remove_edge_helper(AdjListType& adj, n_uid_type n_uid) {
        AdjListItType it = find_node(adj, n_uid);
        assert(it != adj.end());
		e_uid_type e_uid = it->second;
        std::swap(*it, adj.back());
        adj.pop_back();
        return e_uid;
    }

public:

    //
    // CONSTRUCTORS AND DESTRUCTOR
    //

    /** Construct an empty graph. */
    Graph() = default;

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
        Node() = delete;

        /** Return this node's position. */
        const Point& position() const {
            return graph_->nodePool_.info(uid_).p;
        }

        Point& position() {
            return graph_->nodePool_.info(uid_).p;
        }

        /** Return this node's index, a number in the range [0, graph_size). */
        size_type index() const {
            return graph_->nodePool_.info(uid_).idx;
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
            return graph_->nodePool_.info(uid_).v;
        }

        /** Return the value of the node, const version */
        const node_value_type& value() const {
            return graph_->nodePool_.info(uid_).v;
        }

        /** Return the degree of the node */
        typename size_type::IntType degree() const {
            return graph_->nodePool_.info(uid_).adjList.size();
        }

        /** Return the beginning position of incident_iterator */
        incident_iterator edge_begin() const {
            return IncidentIterator(graph_, uid_, true);
        }

        /** Return the end position of incident_iterator */
        incident_iterator edge_end() const {
            return IncidentIterator(graph_, uid_, false);
        }

    private:
        friend class Graph;
        friend class incidentIterator;

        /** Constructs a Node corresponding to given index and graph */
        Node(const Graph* graph, n_uid_type uid)
            : graph_(const_cast<Graph*>(graph)), uid_(uid) {}

        // Pointer back to the Graph container
        Graph *graph_;
        // The element's index in the Graph container
        n_uid_type uid_;
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
        Edge() = delete;

        /** Return a node of this Edge */
        Node node1() const {
            return Node(graph_, info_.n1);
        }

        /** Return the other node of this Edge */
        Node node2() const {
            return Node(graph_, info_.n2);
        }

        /** Test whether this edge and @a e are equal.
         *
         * Equal edges represent the same undirected edge between two nodes.
         */
        bool operator==(const Edge& e) const {
            return uid_ == e.uid_
                && graph_ == e.graph_;
        }

        /** Test whether this edge is less than @a e in a global order.
         *
         * This ordering function is useful for STL containers such as
         * std::map<>. It need not have any interpretive meaning.
         */
        bool operator<(const Edge& e) const {
            return (graph_ < e.graph_)
                || (graph_ == e.graph_ && uid_ < e.uid_);
        }

        /** Return the length of the edge */
        double length() {
            return norm(graph_->nodePool_.info(info_.n1).p - graph_->nodePool_.info(info_.n2).p);
        }


        /** Return this edge's value */
        edge_value_type& value() {
            return info_.v;
        }

        const edge_value_type& value() const {
            return info_.v;
        }

    private:
        // Allow Graph to access Edge's private member data and functions.
        friend class Graph;

        // Constructs an edge given a graph pointer and two node uids
        explicit Edge(const Graph* graph, e_uid_type uid)
            : graph_(const_cast<Graph*>(graph)), uid_(uid),
              info_(graph_->edgePool_.info(uid)) {}

        // Pointer back to the Graph contrainer
        Graph *graph_;
        // The edge's uid
        e_uid_type uid_;
        // The underlying EdgeInfo
        EdgeInfo &info_;
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
        NodeIterator() = delete;

        /** Deference the iterator */
        Node operator*() const {
            return Node(graph_, uid_);
        }

        /** Advance to next position and return the new position*/
        node_iterator& operator++() {
            ++uid_;
			fix();
            return *this;
        }

        /** Test whether two iterators are at the same position */
        bool operator==(const node_iterator& rhs) const {
            return uid_ == rhs.uid_ && graph_ == rhs.graph_;
        }

    private:
        friend class Graph;

        NodeIterator(const Graph* graph, bool isBegin): graph_(const_cast<Graph*>(graph)) {
			if (isBegin)
				uid_.v = 0;
			else
				uid_.v = graph_->nodePool_.capacity();
			fix();
		}

		// fix the iterator's position when it's at an invalid node
		void fix() {
			while (!graph_->nodePool_.alive(uid_) &&
				   uid_ < graph_->nodePool_.capacity())
				++uid_;
		}

        // The uid of Node it is pointing to
        n_uid_type uid_;
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
            return Edge(graph_, uid_);
        }

        /** Avanced to next position */
        edge_iterator& operator++() {
            ++uid_;
            fix();
            return *this;
        }

        /** Test whether two EdgeIterators have the same position */
        bool operator==(const edge_iterator& eit) const {
            return uid_ == eit.uid_
                && graph_ == eit.graph_;
        }

    private:
        friend class Graph;
        e_uid_type uid_;
        // Pointer back to the Graph contrainer
        Graph *graph_;

        void fix() {
            while ((!graph_->edgePool_.alive(uid_))
                   && uid_ < graph_->edgePool_.capacity()) {
                ++uid_;
            }
        }

        // Return the begin position if isBegin == true,
        // otherwise return the end position
        EdgeIterator(const Graph* graph, bool isBegin):
            graph_(const_cast<Graph*>(graph)) {
            if (isBegin)
                uid_.v = 0;
            else
                uid_.v = graph_->edgePool_.capacity();
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

        using AdjListType = typename NodeInfo::AdjListType;


        IncidentIterator() {}

        /** Deference the iterator */
        Edge operator*() const {
            return Edge(graph_, pos_->second);
        }

        /** Return the other endpoint of the edge */
        Node node2() const {
            return Node(graph_, pos_->first);
        }

		/** Index of the other endpoint of the edge */
		size_type index() const {
			return graph_->uid2idx(pos_->first);
		}

        /** Advanced to next position */
        incident_iterator& operator++() {
            ++pos_;
            return *this;
        }

        /** Test whether two iterators are pointing to the same position */
        bool operator==(const incident_iterator& iit) const {
            return pos_ == iit.pos_
                && graph_ == iit.graph_;
        }

    private:
        friend class Node;

        // The surrounding graph
        Graph *graph_;

        // The underlying iterator
        typename AdjListType::iterator pos_;

        // if (isBegin) then point to the begin position,
        // else point to the end position
        IncidentIterator(const Graph* graph, n_uid_type uid, bool isBegin)
            : graph_(const_cast<Graph*>(graph)) {
            if (isBegin)
                pos_ = graph_->nodePool_.info(uid).adjList.begin();
            else
                pos_ = graph_->nodePool_.info(uid).adjList.end();
        }
    };


    /** Return the number of nodes in the graph.
     *
     * Complexity: O(1).
     */
    size_type size() const {
        return size_type(nodePool_.size());
    }

    /** Synonym for size(). */
    size_type num_nodes() const {
        return size();
    }

    /** Translate uid to index */
    size_type uid2idx(n_uid_type uid) const {
        return nodePool_.info(uid).idx;
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
        n_uid_type newUid = nodePool_.add(NodeInfo(position, nValue, size()));
        idx2uid_.push_back(newUid);
		assert(size() == idx2uid_.size());
        return Node(this, newUid);
    }

    /** Remove a node from the graph.
     * @param[in] n The node to be removed
     * @post Node @a n along with all its edges are removed from the graph
     * @post new num_nodes() == old num_nodes() - 1 if @return is true
     *       new num_nodes() == old num_nodes() if @return is false
     * @return Whether the node is an valid node of the graph (before removal)
     *
     * Amortized time complexity is O(sum of degrees of all @a n's neighbors)
     * The indices of the remaining nodes may be changed.
     * All existing node_iterators at @a n are invalidated, and
     * all Node objects corresponding to @a n are invalidated
     */
    bool remove_node(const Node& n) {
        if (!has_node(n)) return false;

        auto &nAdj = nodePool_.info(n.uid_).adjList;
        // Remove all edges associated to this node
        for (auto j = nAdj.begin(); j != nAdj.end(); ++j) {
            // Adjacent list of node2
            auto &n2Adj = nodePool_.info(j->first).adjList;
            edgePool_.remove(remove_edge_helper(n2Adj, n.uid_));
        }

        // Reindex nodes
        idx2uid_[n.index()] = idx2uid_.back();
        nodePool_.info(idx2uid_[n.index()]).idx = n.index();
        idx2uid_.pop_back();

        // Remove node from the pool
        nodePool_.remove(n.uid_);

        return true;
    }

    /** Remove a node from the graph
     * @param[in] n_it A node_iterator at the node to be removed
     * @post The node associated to @a n_it along with all its edges are removed
     * from the graph
     * @post new num_nodes() == old num_nodes() - 1 if @return is true
     *       new num_nodes() == old num_nodes() if @return is false
     * @return Whether the node is an valid node of the graph (before removal)
     *	 
     * Amortized time complexity is O(sum of degrees of all the removed node's neighbors)
     * The indices of the remaining nodes may be changed.
     * All existing node_iterators at the removed node are invalidated, and
     * all Node objects corresponding to @a n are invalidated
     */
    bool remove_node(node_iterator n_it) {
        return NodeIterator(this, remove_node(*n_it));
    }

    /** Remove an edge from the graph
     * @param[in] n1 one node of the edge
     * @param[in] n2 the other node of the edge
     * @pre @a n1 and @a n2 both belong to this graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1 if @return is true
     * @post new num_edges() == old num_edges() if @return is false
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(@a n1.degree() + @a n2.degree())
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(const Node& n1, const Node& n2) {
        if (has_edge(n1, n2)) {
            auto &adj1 = nodePool_.info(n1.uid_).adjList;
            auto &adj2 = nodePool_.info(n2.uid_).adjList;
            remove_edge_helper(adj1, n2.uid_);
            edgePool_.remove(remove_edge_helper(adj2, n1.uid_));
            return true;
        }
        else
            return false;
    }

    /** Remove an edge from the graph
     * @param[in] e the edge to be removed
     * @pre The two endponts of @a e belong to the graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1 if @return is true
	 * @post new num_edges() == old num_edges() if @return is false
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(sum of the two endpoints' degrees)
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(const Edge& e) {
        return remove_edge(e.node1(), e.node2());
    }

    /** Remove an edge from the graph
     * @param[in] e_it an edge_iterator at the edge to be removed
     * @pre The two endpoints of the underlying edge belongs to the graph
     * @post The edge is removed from the graph
     * @post new num_edges() == old num_edges() - 1 if @return is false
	 * @post new num_edges() == old num_edges() if @return is false
     * @return whether the edge is in the graph (before removal)
     *
     * Amortized time complexity is O(sum of the two endpoints' degrees)
     * Upon success, any existing incident_iterator or edge_iterator at the input edge
     * is invalidated
     */
    bool remove_edge(edge_iterator e_it) {
        return remove_edge(*e_it);
    }


    /** Determine if a Node belongs to this Graph
     * @return True if @a n is currently a Node of this Graph
     *
     * Complexity: O(1).
     */
    bool has_node(const Node& n) const {
        return n.graph_ == this
            && nodePool_.alive(n.uid_);
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
        return size_type(edgePool_.size());
    }

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

    /** Test whether two nodes are connected by an edge.
     * @pre @a a and @a b are valid nodes of this graph
     * @return True if for some @a i, edge(@a i) connects @a a and @a b.
     *
     * Complexity: No more than O(num_nodes() + num_edges()), hopefully less
     */
    bool has_edge(const Node& a, const Node& b) const {
        assert(has_node(a) && has_node(b));
        auto &a_adj = nodePool_.info(a.uid_).adjList;
        return find_node(a_adj, b.uid_) != a_adj.end();
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
        assert(has_node(a) && has_node(b));
        auto &a_adj = nodePool_.info(a.uid_).adjList;
        auto it = find_node(a_adj, b.uid_);
        if (it == a_adj.end()) {
            e_uid_type newUid = edgePool_.add(EdgeInfo(a.uid_, b.uid_));
            a_adj.push_back({ b.uid_, newUid });
            nodePool_.info(b.uid_).adjList.push_back({ a.uid_, newUid });
            return Edge(this, newUid);
        }
        else
            return Edge(this, it->second);
    }

    /** Remove all nodes and edges from this graph.
     * @post num_nodes() == 0 && num_edges() == 0
     *
     * Invalidates all outstanding Node and Edge objects.
     */
    void clear() {
        nodePool_.clear();
        edgePool_.clear();
        idx2uid_.clear();
    }

    /** Return an iterator pointing to the first node */
    node_iterator node_begin() const {
        return node_iterator(this, 1);
    }
    /** Return an iterator pointing to the next position of the last node */
    node_iterator node_end() const {
        return node_iterator(this, 0);
    }
    /** Return an iterator pointing to the first edge */
    edge_iterator edge_begin() const {
        return edge_iterator(this, true);
    }
    /** Return an iterator pointing to the next position of the last edge */
    edge_iterator edge_end() const {
        return edge_iterator(this, false);
    }

private:
    ObjPool<NodeInfo> nodePool_;
    ObjPool<EdgeInfo> edgePool_;
    std::vector<n_uid_type> idx2uid_;
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
