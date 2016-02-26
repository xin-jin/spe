#pragma once

#include <CME212/Util.hpp>
#include "graph/EdgeIterator.hpp"

/** @class Graph::Edge
 * @brief Class representing the graph's edges.
 *
 * Edges are order-insensitive pairs of nodes. Two Edges with the same nodes
 * are considered equal if they connect the same nodes, in either order.
 */
template <typename V, typename E>
class Edge: private totally_ordered<Edge<V, E>> {
 public:
    using e_uid_type = typename Graph<V, E>::e_uid_type;
    using EdgeInfo = typename Graph<V, E>::EdgeInfo;
    using Node = typename Graph<V, E>::Node;
    using EdgeIterator = typename Graph<V, E>::EdgeIterator;
    using IncidentIterator = typename Graph<V, E>::IncidentIterator;
    using edge_value_type = E;

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
    friend class Graph<V, E>;
    friend EdgeIterator;
    friend IncidentIterator;

    // Constructs an edge given a graph pointer and two node uids
    explicit Edge(const Graph<V, E>* graph, e_uid_type uid)
                  : graph_(const_cast<Graph<V, E>*>(graph)), uid_(uid),
                  info_(graph_->edgePool_.info(uid)) {}

    // Pointer back to the Graph contrainer
    Graph<V, E> *graph_;
    // The edge's uid
    e_uid_type uid_;
    // The underlying EdgeInfo
    EdgeInfo &info_;
};
