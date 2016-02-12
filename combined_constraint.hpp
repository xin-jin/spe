#ifndef COMBINED_CONSTRAINT_HPP
#define COMBINED_CONSTRAINT_HPP

#include <utility>
#include "Graph.hpp"

/** A class representing combined constraints */
template <typename C1, typename ...Cn>
class CombinedConstraint {
public:
    template <typename Graph>
    void operator()(Graph& graph, double t) {
        c1_(graph, t);
        cn_(graph, t);
    }

    CombinedConstraint(C1&& c1, Cn&& ...cn):
        c1_(c1), cn_(std::forward<Cn>(cn)...) {}

private:
    C1 c1_;
    CombinedConstraint<Cn...> cn_;
};

// Boundary case
template <typename C>
struct CombinedConstraint<C> {
public:
    template <typename Graph>
    void operator()(Graph& graph, double t) {
        c_(graph, t);
    }

    CombinedConstraint(C&& c): c_(c) {}

private:
    C c_;
};

// Alias of CombinedConstraint
template <typename ...C>
using combined_constraint = CombinedConstraint<C...>;


/** Given an arbitrary number of constraints,
 * return a combined constraint incorporating all
 * of them
 */
template <typename ...C>
auto makeCombinedConstraint(C&& ...c) {
    return combined_constraint<C...>(std::forward<C>(c)...);
}

/** Wrapper for makeCombinedConstraint */
template <typename ...C>
auto make_combined_constraint(C&& ...c) {
    return makeCombinedConstraint(std::forward<C>(c)...);
}


#endif // COMBINED_CONSTRAINT_HPP
