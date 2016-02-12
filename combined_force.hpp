#ifndef COMBINED_FORCE_HPP
#define COMBINED_FORCE_HPP

/** @file combined_force.hpp
 * @brief Provide a function to combine an arbitrary number of forces
 */

#include <utility>
#include "CME212/Point.hpp"

/** A force class representing combined forces */
template <typename F1, typename ...Fn>
class CombinedForce {
public:
	template <typename Node>
	Point operator()(Node n, double t) {
		return f1_(n, t) + fn_(n, t);
	}
	
	CombinedForce(F1&& f1, Fn&& ...fn):
		f1_(f1), fn_(std::forward<Fn>(fn)...) {}
	
private:		
	F1 f1_;
	CombinedForce<Fn...> fn_;
};

// Boundary case
template <typename F>
class CombinedForce<F> {
public:
	template <typename Node>
	Point operator()(Node n, double t) {
		return f_(n, t);
	}	

	CombinedForce<F>(F&& f): f_(f) {}
	
private:	
	F f_;
};

// Alias of CombinedForce
template <typename ...F>
using combined_force = CombinedForce<F...>;


/** Given an arbitrary number of forces, return
 * a combined force.
 */
template <typename ...F>
auto makeCombinedForce(F&& ...f) {
	return combined_force<F...>(std::forward<F>(f)...);
}

/** Wrapper for makeCombinedForce */
template <typename ...F>
auto make_combined_force(F&& ...f) {
	return makeCombinedForce(std::forward<F>(f)...);
}


#endif // COMBINED_FORCE_HPP
