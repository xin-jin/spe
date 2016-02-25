#pragma once

#include <CME212/Util.hpp>


template <typename V, typename E>
class Graph;

/** @class Graph::EdgeIterator
 * @brief Iterator class for edges. A forward iterator. */
template <typename V, typename E>
class EdgeIterator: private totally_ordered<EdgeIterator<V, E>> {
public:
    // These type definitions help us use STL's iterator_traits.
	using Edge = typename Graph<V, E>::Edge;
	using edge_iterator = typename Graph<V, E>::edge_iterator;
	using e_uid_type = typename Graph<V, E>::e_uid_type;
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
    friend class Graph<V, E>;
    e_uid_type uid_;
    // Pointer back to the Graph contrainer
    Graph<V, E> *graph_;

    void fix() {
        while ((!graph_->edgePool_.alive(uid_))
               && uid_ < graph_->edgePool_.capacity()) {
            ++uid_;
        }
    }

    // Return the begin position if isBegin == true,
    // otherwise return the end position
    EdgeIterator(const Graph<V, E>* graph, bool isBegin):
        graph_(const_cast<Graph<V, E>*>(graph)) {
        if (isBegin)
            uid_.v = 0;
        else
            uid_.v = graph_->edgePool_.capacity();
        fix();
    }
};
