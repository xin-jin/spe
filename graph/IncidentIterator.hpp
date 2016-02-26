#pragma once

#include "CME212/Util.hpp"


/** @class Graph::IncidentIterator
 * @brief Iterator class for edges incident to a node. A forward iterator. */
template <typename V, typename E>
class IncidentIterator: private totally_ordered<IncidentIterator<V, E>> {
public:
	using size_type = typename Graph<V, E>::size_type;
	using n_uid_type = typename Graph<V, E>::n_uid_type;
	using Node = typename Graph<V, E>::Node;
	using Edge = typename Graph<V, E>::Edge;
	using NodeInfo = typename Graph<V, E>::NodeInfo;
	using AdjListType = typename NodeInfo::AdjListType;
	using incident_iterator = IncidentIterator;

	
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
    friend Node;

    // The surrounding graph
    Graph<V, E> *graph_;

    // The underlying iterator
    typename AdjListType::iterator pos_;

    // if (isBegin) then point to the begin position,
    // else point to the end position
    IncidentIterator(const Graph<V, E>* graph, n_uid_type uid, bool isBegin)
        : graph_(const_cast<Graph<V, E>*>(graph)) {
        if (isBegin)
            pos_ = graph_->nodePool_.info(uid).adjList.begin();
        else
            pos_ = graph_->nodePool_.info(uid).adjList.end();
    }
};
