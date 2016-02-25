#ifndef COMBINED_CONSTRAINT_HPP
#define COMBINED_CONSTRAINT_HPP

#include <utility>
#include "graph/Graph.hpp"

/** A class representing combined constraints */
template <typename ...>
class CombinedConstraint {
public:
    template <typename Graph>
    void operator()(__attribute__((unused)) Graph&,
                    __attribute__((unused)) double) {}
};

template <typename C1, typename ...Cn>
class CombinedConstraint<C1, Cn...> {
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

// Alias of CombinedConstraint
template <typename ...C>
using combined_constraint = CombinedConstraint<C...>;


/** Given an arbitrary number of constraints,
 * return a combined constraint incorporating all
 * of them. If no argument is supplied, it returns
 * a void constraint (a functor that does nothing).
 */
template <typename ...C>
auto makeCombinedConstraint(C&& ...c) -> combined_constraint<C...> {
    return combined_constraint<C...>(std::forward<C>(c)...);
}

/** Wrapper for makeCombinedConstraint */
template <typename ...C>
auto make_combined_constraint(C&& ...c) -> combined_constraint<C...> {
    return makeCombinedConstraint(std::forward<C>(c)...);
}


#endif // COMBINED_CONSTRAINT_HPP
