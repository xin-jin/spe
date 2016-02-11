#ifndef COMBINED_FORCE_HPP
#define COMBINED_FORCE_HPP

#include <utility>
#include "CME212/Point.hpp"


template <typename F1, typename ...Fn>
class CombinedForce;

template <typename F>
class CombinedForce<F>;

template <typename ...F>
using combined_force = CombinedForce<F...>;


/** Return a combined force, function version */
template <typename ...F>
auto makeCombinedForce(F&& ...f) {
	return combined_force<F...>(f...);
}

/** Wrapper for makeCombinedForce */
template <typename ...F>
auto make_combined_force(F&& ...f) {
	return makeCombinedForce(std::forward<F>(f)...);
}


template <typename F1, typename ...Fn>
class CombinedForce {
public:
	template <typename Node>
	Point operator()(Node n, double t) {
		return f1_(n, t) + fn_(n, t);
	}
	
	CombinedForce(F1 f1, Fn ...fn): f1_(f1), fn_(fn...) {}
	
private:		
	F1 f1_;
	CombinedForce<Fn...> fn_;
};

template <typename F>
class CombinedForce<F> {
public:
	template <typename Node>
	Point operator()(Node n, double t) {
		return f_(n, t);
	}	

	CombinedForce<F>(F f): f_(f) {}
	
private:	
	F f_;
};


#endif // COMBINED_FORCE_HPP
