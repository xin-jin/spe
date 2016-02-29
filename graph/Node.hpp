#pragma once

#include "CME212/Util.hpp"
#include "CME212/Point.hpp"
#include "graph/NodeIterator.hpp"
#include "graph/IncidentIterator.hpp"


template <typename V, typename E>
class Graph;

/** @class Graph::Node
 * @brief Class representing the graph's nodes.
 *
 * Node objects are used to access information about the Graph's nodes.
 */
template <typename V, typename E>
class Node: private totally_ordered<Node<V, E>> {
 public:
    using size_type = typename Graph<V, E>::size_type;
    using n_uid_type = typename Graph<V, E>::n_uid_type;
    using node_value_type = V;
    using incident_iterator = typename Graph<V, E>::incident_iterator;
    using IncidentIterator = incident_iterator;
    using NodeIterator = typename Graph<V, E>::NodeIterator;
    using Edge = typename Graph<V, E>::Edge;

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
    friend class Graph<V, E>;
    friend Edge;
    friend NodeIterator;
    friend incident_iterator;

    /** Constructs a Node corresponding to given index and graph */
    Node(const Graph<V, E>* graph, n_uid_type uid)
        : graph_(const_cast<Graph<V, E>*>(graph)), uid_(uid) {}

    // Pointer back to the Graph container
    Graph<V, E> *graph_;
    // The element's index in the Graph container
    n_uid_type uid_;
};
