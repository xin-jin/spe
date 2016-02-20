#ifndef CME212_SUBGRAPH_HPP
#define CME212_SUBGRAPH_HPP

#include <iterator>
#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "CME212/Util.hpp"

#include "Graph.hpp"

/** An iterator that skips over elements of another iterator based on whether
 * those elements satisfy a predicate.
 *
 * Given an iterator range [@a first, @a last) and a predicate @a pred,
 * this iterator models a filtered range such that all i with
 * @a first <= i < @a last and @a pred(*i) appear in order of the original range.
 */
template <typename Pred, typename It>
class filter_iterator
    : private equality_comparable<filter_iterator<Pred,It>> {
 public:
  // Get all of the iterator traits and make them our own
  typedef typename std::iterator_traits<It>::value_type        value_type;
  typedef typename std::iterator_traits<It>::pointer           pointer;
  typedef typename std::iterator_traits<It>::reference         reference;
  typedef typename std::iterator_traits<It>::difference_type   difference_type;
  typedef typename std::input_iterator_tag                     iterator_category;

  typedef filter_iterator<Pred,It> self_type;

  /** Constructor
    * @param[in] p predicate for nodes
    * @param[in] first iterator at first node position to consider
    * @param[in] end iterator one past last node position to consider
    * @pre [@a first, @a end) is a valid iterator range
    * @pre operator() p supports iterator value type
    * @post p(**this) or *this == end
    */
  filter_iterator(const Pred& p, const It& first, const It& end)
      : p_(p), begin_(first), it_(first), end_(end) {
    fix();
  }

  /** Dereference operator 
    * @pre valid iterator not equal to end
    * @return value at filter iterator position
    * @post p(**this)
    */
  value_type operator*() const {
    return *it_;
  }

  /** Increment operator
    * @pre valid iterator not equal to end
    * @return filter iterator with incremented position
    * @post p(**this)
    */
  self_type& operator++() {
    ++it_;
    fix();
    return *this;
  }

  /** Equivalence comparator
    * @param[in] it filter iterator to compare to
    * @pre *this and @a it are valid iterators with same predicate
    * @return boolean indicating eqivalence
    */
  bool operator==(const self_type& f_it) const {
    return ((it_ == f_it.it_) && (end_ == f_it.end_));
  }

 private:
  Pred p_;
  It begin_;
  It it_;
  It end_;

  /** Fix conditions of EdgeIterator to satisfy invariants
    *
    * Assure predicate is fulfilled at current position
    * If not, increment until end reached
    */
  void fix() {
    while (!p_(*it_)) {
      if (it_ == end_)
        return;
      ++it_;
    }
  }
};

/** Helper function for constructing filter_iterators.
 *
 * Usage:
 * // Construct an iterator that filters odd values out and keeps even values.
 * std::vector<int> a = ...;
 * auto it = make_filtered(a.begin(), a.end(), [](int k) {return k % 2 == 0;});
 */
template <typename Pred, typename Iter>
filter_iterator<Pred,Iter> make_filtered(const Iter& it, const Iter& end,
                                         const Pred& p) {
  return filter_iterator<Pred,Iter>(p, it, end);
}

/** Samples node with supplied probability, or if all neighbors 
  * have been sampled. Creates groups of complete subgraphs
  */
struct ClusterPredicate {
  /** Constructor
    * @param[in] pr probability of sampling node
    * @pre 0 <= pr <= 1
    */
  ClusterPredicate(float pr) : pr_(pr) {};
  ClusterPredicate() : pr_(0.1) {};
  /** Predicate operator 
    * @param[in] n node to "predicate upon"
    * @return boolean indicating if node should be sampled
    * @post return true if all of @a n's neighbors have been sampled
    * @post if not all of @ n's neighbors have been sampled, return true
    *       with probability pr
    */
  template <typename NODE>
  bool operator()(const NODE& n) {
    float r = float(std::rand()) / float(RAND_MAX);
    if (r < pr_) {
      vis_.insert(n.index());
      return true;
    }
    else {
      for (auto it = n.edge_begin(); it != n.edge_end(); ++it) {
        if (!((bool) vis_.count((*it).node2().index())))
          return false;
      }
      vis_.insert(n.index());
      return true;
    }
  }
 private:
  float pr_;
  typedef unsigned size_type;
  std::unordered_set<size_type> vis_ = std::unordered_set<size_type>();
};

/** Takes all nodes inside an infinity norm ball centered at 0 */
struct InfNormPredicate {
  template <typename NODE>
  /** Predicate operator
    * @param[in] n node to "predicate upon"
    * @return boolean indicating if node should be sampled
    * @post return true if L_1 norm of @n is less than 0.2
    */
  bool operator()(const NODE& n) {
    auto l_inf = std::max(std::abs(n.position().x),
                          std::abs(n.position().y));
    l_inf = std::max(l_inf, std::abs(n.position().z));
    return l_inf < 0.2;
  }
};

/** Test predicate for HW1 #4 */
struct SlicePredicateLeft {
  template <typename NODE>
  bool operator()(const NODE& n) {
    return n.position().x < 0;
  }
};

/** Test predicate for HW1 #4 (modified for right side) */
struct SlicePredicateRight {
  template <typename NODE>
  bool operator()(const NODE& n) {
    return n.position().x >= 0;
  }
};

#endif // CME212_SUBGRAPH_HPP
