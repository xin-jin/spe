#include "Graph.hpp"

using size_type = typename Graph<int>::size_type;
template <typename V>
using Node = typename Graph<V>::Node;

////////////////////////////////////
// Implementations of Graph methods //
////////////////////////////////////


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
// Implementations of Node methods //
/////////////////////////////////////

template <typename V>
Graph<V>::Node::Node() {}

template <typename V>
Graph<V>::Node::Node(const Graph* graph, size_type index)
	: index_(index), graph_(const_cast<Graph*>(graph)) {}

template <typename V>
const Point& Graph<V>::Node::position() const {
	return graph_->points_[index_];
}

template <typename V>
size_type Graph<V>::Node::index() const {
	return index_;
}

template <typename V>
bool Graph<V>::Node::operator==(const Node& n) const {
	return (graph_ == n.graph_ && index_ == n.index_);
}

template <typename V>
bool Graph<V>::Node::operator<(const Node& n) const {
	// Order is defined as dictionary order in (graph_, index_)
	return (graph_ < n.graph_ || (graph_ == n.graph_ && index_ < n.index_));
}

template <typename V>
node_value_type& Graph<V>::Node::value() {
	return graph_->nValues_[index_];
}

template <typename V>
const node_value_type& Graph<V>::Node::value() const {
	return graph_->nValues_[index_];
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
// Implementations of EdgeIterator methods //
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
