/**
 * @file mtl_test.cpp
 * Test script for interfacing with MTL4 and it's linear solvers.
 */

#include <cstddef>

#include "boost/numeric/mtl/mtl.hpp"
#include <boost/numeric/itl/itl.hpp>

using size_t = std::size_t;

template <size_t n>
struct IdentityMatrix {
    /** Helper function to perform multiplication. Allows for delayed
     * evaluation of results;
     * Assign::apply(a, b) resolves to an assignment operations such as
     * a += b, a -= b, or a = b
     * @pre @a size(v) == size(w)
     */
    template <typename VectorIn, typename VectorOut, typename Assign>
    void mult(const VectorIn& v, VectorOut& w, Assign) const {
        assert(size(v) == size(w));
        for (size_t i = 0; i < size(v); ++i)
            Assign::apply(w[i], v[i]);
    }

    /** Matvec forwards to MTL's lazy mat_cvec_multiplier operator */
    template <typename Vector>
    typename mtl::vec::mat_cvec_multiplier<IdentityMatrix, Vector>
    operator*(const Vector& v) const {
        return {*this, v};
    }
};

/** The number of elements in the matrix */
template <size_t n>
inline size_t size(const IdentityMatrix<n>& A);
/** The number of rows in the matrix */
template <size_t n>
inline size_t num_rows(const IdentityMatrix<n>& A);
/** The number of columns in the matrix */
template <size_t n>
inline size_t num_cols(const IdentityMatrix<n>& A);

/** Traits that MTL uses to determine properties of our IdentityMatrix */
namespace mtl {
    namespace ashape {
        /** Define Identity Matrix to be a non-scalar type */
        template<size_t n>
        struct ashape_aux<IdentityMatrix<n>> {
            typedef nonscal type;
        };
    }

    /** IdentityMatrix implements the Collection concept with value_type and size_type */
    template<size_t n>
    struct Collection<IdentityMatrix<n>> {
        typedef double      value_type;
        typedef unsigned    size_type;
    };
}


int main()
{
	using matrix_type = IdentityMatrix<10>;
    matrix_type I;
    mtl::dense_vector<double> b(10), x(10);
    iota(b);
	// preconditioner
	itl::pc::identity<matrix_type> L(I);
	// iteration object
	itl::basic_iteration<double> iter(b, 500, 1.e-6);
	// Solve Ix = b using conjugate gradient method
	itl::cg(I, x, b, L, iter);
	// Print result
	std::cout << x << std::endl;

    return 0;
}


/////////////////////
// Implementations //
/////////////////////

template <size_t n>
inline size_t size(const IdentityMatrix<n>& A) {
    (void) A;
    return n;
}

template <size_t n>
inline size_t num_rows(const IdentityMatrix<n>& A) {
    (void) A;
    return n;
}

template <size_t n>
inline size_t num_cols(const IdentityMatrix<n>& A) {
    (void) A;
    return n;
}
