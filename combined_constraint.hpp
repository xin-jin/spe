#ifndef COMBINED_CONSTRAINT_HPP
#define COMBINED_CONSTRAINT_HPP

#include "Graph.hpp"


template <typename C1, typename ...Cn>
struct CombinedConstraint {
	template <typename Graph>
	void operator()(Graph& graph, double t) {
		c1(graph, t);
		cn(graph, t);
	}

	C1 c1;
	CombinedConstraint<Cn...> cn;
};

template <typename C>
struct CombinedConstraint<C> {
	template <typename Graph>
	void operator()(Graph& graph, double t) {
		c(graph, t);
	}

	C c;
};

template <typename ...C>
using combined_constraint = CombinedConstraint<C...>;

/** Return a combined constraint, function version */
template <typename ...C>
auto makeCombinedConstraint(__attribute__((unused)) C&& ...c) {
	return combined_constraint<C...>();
}

/** Alias for makeCombinedConstraint */
template <typename ...C>
constexpr auto make_combined_constraint = makeCombinedConstraint<C...>;


#endif // COMBINED_CONSTRAINT_HPP
