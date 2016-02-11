#include "CME212/Point.hpp"

// TODO: Write specification
template <typename F1, typename ...Fn>
struct CombinedForce {
	template <typename Node>
	Point operator()(Node n, double t) {
		return f1(n, t) + fn(n, t);
	}

	F1 f1;
	CombinedForce<Fn...> fn;
};

template <typename F>
struct CombinedForce<F> {
	template <typename Node>
	Point operator()(Node n, double t) {
		return f(n, t);
	}

	F f;
};

template <typename Node, typename ...Fn>
using combined_force = CombinedForce<Node, Fn...>;

/** Return a combined force, function version */
template <typename ...F>
auto makeCombinedForce(__attribute__((unused)) F&& ...f) {
	return CombinedForce<F...>();
}

// Alias for makeCombinedForce
template <typename Node, typename ...F>
constexpr auto make_combined_force = makeCombinedForce<Node, F...>;
