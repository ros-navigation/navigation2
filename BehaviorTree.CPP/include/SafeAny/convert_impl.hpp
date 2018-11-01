#ifndef CONVERT_IMPL_HPP
#define CONVERT_IMPL_HPP

#include <type_traits>
#include <exception>
#include "simple_string.hpp"

namespace SafeAny
{
namespace details
{
template <typename BoolCondition>
using EnableIf = typename std::enable_if<BoolCondition::value, void>::type;

template <typename T>
struct is_integer
  : std::integral_constant<bool, std::is_integral<T>::value && !std::is_same<T, bool>::value &&
                                     !std::is_same<T, char>::value>
{
};

template <typename From, typename To>
struct is_same_real
  : std::integral_constant<bool, std::is_same<From, To>::value && std::is_floating_point<To>::value>
{
};

template <typename From, typename To>
struct is_safe_integer_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     sizeof(From) < sizeof(To) &&
                                     std::is_signed<From>::value == std::is_signed<To>::value>
{
};

template <typename T>
struct is_convertible_type
  : std::integral_constant<bool, std::is_integral<T>::value || std::is_floating_point<T>::value ||
                                     std::is_same<T, bool>::value || std::is_same<T, char>::value ||
                                     std::is_same<T, std::string>::value ||
                                     std::is_same<T, SimpleString>::value || std::is_enum<T>::value>
{
};

template <typename From, typename To>
struct float_conversion : std::integral_constant<bool, std::is_floating_point<From>::value &&
                                                           std::is_floating_point<To>::value &&
                                                           !std::is_same<From, To>::value>
{
};

template <typename From, typename To>
struct unsigned_to_smaller_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     (sizeof(From) > sizeof(To)) && !std::is_signed<From>::value &&
                                     !std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct signed_to_smaller_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     (sizeof(From) > sizeof(To)) && std::is_signed<From>::value &&
                                     std::is_signed<To>::value>
{
};

//---------------------------
template <typename From, typename To>
struct signed_to_smaller_unsigned_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     sizeof(From) >= sizeof(To) && std::is_signed<From>::value &&
                                     !std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct signed_to_larger_unsigned_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     sizeof(From) < sizeof(To) && std::is_signed<From>::value &&
                                     !std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct unsigned_to_smaller_signed_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     (sizeof(From) >= sizeof(To)) && !std::is_signed<From>::value &&
                                     std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct unsigned_to_larger_signed_conversion
  : std::integral_constant<bool, is_integer<From>::value && is_integer<To>::value &&
                                     sizeof(From) < sizeof(To) && !std::is_signed<From>::value &&
                                     std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct floating_to_signed_conversion
  : std::integral_constant<bool, std::is_floating_point<From>::value && is_integer<To>::value &&
                                     std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct floating_to_unsigned_conversion
  : std::integral_constant<bool, std::is_floating_point<From>::value && is_integer<To>::value &&
                                     !std::is_signed<To>::value>
{
};

template <typename From, typename To>
struct integer_to_floating_conversion
  : std::integral_constant<bool, is_integer<From>::value && std::is_floating_point<To>::value>
{
};

template <typename From, typename To>
inline void checkUpperLimit(const From& from)
{
    if ((sizeof(To) < sizeof(From)) && (from > static_cast<From>(std::numeric_limits<To>::max())))
    {
        throw std::runtime_error("Value too large.");
    }
    else if (static_cast<To>(from) > std::numeric_limits<To>::max())
    {
        throw std::runtime_error("Value too large.");
    }
}

template <typename From, typename To>
inline void checkUpperLimitFloat(const From& from)
{
    if (from > std::numeric_limits<To>::max())
    {
        throw std::runtime_error("Value too large.");
    }
}

template <typename From, typename To>
inline void checkLowerLimitFloat(const From& from)
{
    if (from < -std::numeric_limits<To>::max())
    {
        throw std::runtime_error("Value too small.");
    }
}

template <typename From, typename To>
inline void checkLowerLimit(const From& from)
{
    if (from < std::numeric_limits<To>::min())
    {
        throw std::runtime_error("Value too small.");
    }
}

template <typename From, typename To>
inline void checkTruncation(const From& from)
{
    if (from != static_cast<From>(static_cast<To>(from)))
    {
        throw std::runtime_error("Floating point truncated");
    }
}

//----------------------- Implementation ----------------------------------------------

template <typename SRC, typename DST>
inline typename std::enable_if<!is_convertible_type<DST>::value, void>::type
convertNumber(const SRC&, DST&)
{
    static_assert(is_convertible_type<DST>::value, "Not convertible");
}

template <typename SRC, typename DST>
inline EnableIf<std::is_same<bool, DST>> convertNumber(const SRC& from, DST& target)
{
    target = (from != 0);
}

template <typename SRC, typename DST>
inline EnableIf<std::is_same<SRC, DST>> convertNumber(const SRC& from, DST& target)
{
    target = from;
}

template <typename SRC, typename DST>
inline EnableIf<is_safe_integer_conversion<SRC, DST>> convertNumber(const SRC& from, DST& target)
{
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<float_conversion<SRC, DST>> convertNumber(const SRC& from, DST& target)
{
    checkTruncation<SRC, DST>(from);
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<unsigned_to_smaller_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                        DST& target)
{
    checkUpperLimit<SRC, DST>(from);
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<signed_to_smaller_conversion<SRC, DST>> convertNumber(const SRC& from, DST& target)
{
    checkLowerLimit<SRC, DST>(from);
    checkUpperLimit<SRC, DST>(from);
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<signed_to_smaller_unsigned_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                               DST& target)
{
    if (from < 0)
    {
        throw std::runtime_error("Value is negative and can't be converted to signed");
    }

    checkUpperLimit<SRC, DST>(from);
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<signed_to_larger_unsigned_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                              DST& target)
{
    if (from < 0)
    {
        throw std::runtime_error("Value is negative and can't be converted to signed");
    }

    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<unsigned_to_larger_signed_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                              DST& target)
{
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<unsigned_to_smaller_signed_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                               DST& target)
{
    checkUpperLimit<SRC, DST>(from);
    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<floating_to_signed_conversion<SRC, DST>> convertNumber(const SRC& from, DST& target)
{
    checkLowerLimitFloat<SRC, DST>(from);
    checkLowerLimitFloat<SRC, DST>(from);

    if (from != static_cast<SRC>(static_cast<DST>(from)))
    {
        throw std::runtime_error("Floating point truncated");
    }

    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<floating_to_unsigned_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                         DST& target)
{
    if (from < 0)
    {
        throw std::runtime_error("Value is negative and can't be converted to signed");
    }

    checkLowerLimitFloat<SRC, DST>(from);

    if (from != static_cast<SRC>(static_cast<DST>(from)))
    {
        throw std::runtime_error("Floating point truncated");
    }

    target = static_cast<DST>(from);
}

template <typename SRC, typename DST>
inline EnableIf<integer_to_floating_conversion<SRC, DST>> convertNumber(const SRC& from,
                                                                        DST& target)
{
    checkTruncation<SRC, DST>(from);
    target = static_cast<DST>(from);
}

}   //end namespace details
}   //end namespace details

#endif   // CONVERT_IMPL_HPP
