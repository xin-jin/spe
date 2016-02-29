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
#include "graph/Node.hpp"
#include "graph/Edge.hpp"


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

    typedef Node<V, E> Node;
    /** Synonym for Node (following STL conventions). */
    typedef Node node_type;

    typedef Edge<V, E> Edge;
    /** Synonym for Edge (following STL conventions). */
    typedef Edge edge_type;
	
    /** Type of node iterators, which iterate over all graph edges. */
	typedef NodeIterator<V, E> NodeIterator;
    /** Synonym for NodeIterator */
    typedef NodeIterator node_iterator;

    /** Type of edge iterators, which iterate over all graph edges. */
    typedef EdgeIterator<V, E> EdgeIterator;
    /** Synonym for EdgeIterator */
    typedef EdgeIterator edge_iterator;

    /** Type of incident iterators, which iterate incident edges to a node. */
    typedef IncidentIterator<V, E> IncidentIterator;
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
            n1(n1_), n2(n2_), v(v_) {}

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
	friend Node;
	friend Edge;
	friend NodeIterator;
	friend EdgeIterator;
	friend IncidentIterator;
	
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
