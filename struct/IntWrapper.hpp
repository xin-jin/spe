#ifndef INT_WRAPPER_HPP
#define INT_WRAPPER_HPP

#include <algorithm>


/** A wrapper of int to provide "strongly typed" int. It supports implicit conversion
 * to the underlying int so that it can directly be used as a subscript, but forbids
 * implicit conversion *from* other types.
 *
 * It's also hashable.
 */
template <typename TAG, typename TIntType = unsigned>
struct IntWrapper {
    using IntType = TIntType;

    IntWrapper() = default;
    explicit IntWrapper(IntType v_): v(v_) {}

    operator IntType() const {
        return v;
    }

    IntWrapper& operator++() {
        ++v;
        return *this;
    }

    IntWrapper& operator--() {
        --v;
        return *this;
    }

	IntType value() {
		return v;
	}

    bool operator==(const IntWrapper& rhs) const {
        return v == rhs.v;
    }

    bool operator<(const IntWrapper& rhs) const {
        return v < rhs.v;
    }

    IntType v;
};

template <typename TAG>
struct std::hash<IntWrapper<TAG>> {
	typedef IntWrapper<TAG> argument_type;
	typedef size_t result_type;
	result_type operator()(const argument_type& a) const {
		return std::hash<int>()(a.v);
	}
};


#endif // INT_WRAPPER_HPP
