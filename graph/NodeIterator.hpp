#pragma once

#include "CME212/Util.hpp"
#include "thrust/iterator/transform_iterator.h"
#include "thrust/iterator/counting_iterator.h"
#include "thrust/functional.h"


/** Return the node corresponding to a uid */
template <typename GraphType>
struct Idx2Node: thrust::unary_function<typename GraphType::size_type::IntType,
                                        typename GraphType::Node> {
    using size_type = typename GraphType::size_type;

    Idx2Node(const GraphType* g_): g(g_) {}

    typename GraphType::Node operator()(typename size_type::IntType idx) const {
        return g->node(size_type(idx));
    }

    const GraphType *g;
};

/** @class Graph::NodeIterator
 * @brief Iterator class for nodes. A forward iterator. */
// TODO: cryptic error with GraphType::size_type not found
template <typename GraphType>
using NodeIterator = thrust::transform_iterator<Idx2Node<GraphType>,
                                                thrust::counting_iterator<unsigned>, 
                                                typename GraphType::Node>;

// Inheritance approach has some type traits issue with difference_type
// template <typename V, typename E>
// struct NodeIterator:
//     thrust::transform_iterator<Idx2Node<Graph<V, E>>,
//                                thrust::counting_iterator<
//                                 typename Graph<V, E>::size_type::IntType>,
//                                typename Graph<V, E>::Node> {
//     using Graph = ::Graph<V, E>;
//     using size_type = typename Graph::size_type;
//     using super_t = thrust::transform_iterator<Idx2Node<Graph>,
//                                                thrust::counting_iterator<
//                                                 typename Graph::size_type::IntType>,
//                                                typename Graph::Node>;

//     NodeIterator(const Graph* g, typename Graph::size_type idx):
//         super_t(thrust::make_counting_iterator(idx.v),
//                 Idx2Node<Graph>(g)) {}
// };

/** Define STL-style synonym for NodeIterator */
template <typename Graph>
using node_iterator = NodeIterator<Graph>;


// Deprecated
// /** @class Graph::NodeIterator
//  * @brief Iterator class for nodes. A forward iterator. */
// template <typename V, typename E>
// class NodeIterator: private totally_ordered<NodeIterator<V, E>> {
//  public:
//  using Node = typename Graph<V, E>::Node;
//  using n_uid_type = typename Graph<V, E>::n_uid_type;

//     // These type definitions help us use STL's iterator_traits.
//     /** Element type. */
//     typedef Node value_type;
//     /** Type of pointers to elements. */
//     typedef Node* pointer;
//     /** Type of references to elements. */
//     typedef Node& reference;
//     /** Iterator category. */
//     typedef std::input_iterator_tag iterator_category;
//     /** Difference between iterators */
//     typedef std::ptrdiff_t difference_type;

//     /** Construct an invalid NodeIterator. */
//     NodeIterator() = delete;

//     /** Deference the iterator */
//     value_type operator*() const {
//         return value_type(graph_, uid_);
//     }

//     /** Advance to next position and return the new position*/
//     NodeIterator& operator++() {
//         ++uid_;
//         fix();
//         return *this;
//     }

//     /** Test whether two iterators are at the same position */
//     bool operator==(const NodeIterator& rhs) const {
//         return uid_ == rhs.uid_ && graph_ == rhs.graph_;
//     }

//  private:
//     friend class Graph<V, E>;

//     NodeIterator(const Graph<V, E>* graph, bool isBegin): graph_(const_cast<Graph<V, E>*>(graph)) {
//         if (isBegin)
//             uid_.v = 0;
//         else
//             uid_.v = graph_->nodePool_.capacity();
//         fix();
//     }

//     // fix the iterator's position when it's at an invalid node
//     void fix() {
//         while (!graph_->nodePool_.alive(uid_) &&
//                uid_ < graph_->nodePool_.capacity())
//             ++uid_;
//     }

//     // The uid of Node it is pointing to
//     n_uid_type uid_;
//     // Pointer back to the Graph contrainer
//     Graph<V, E> *graph_;
// };
