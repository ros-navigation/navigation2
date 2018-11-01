#ifndef SAFE_ANY_VARNUMBER_H
#define SAFE_ANY_VARNUMBER_H

#include <exception>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <string>
#include <cstring>
#include <type_traits>
#include "any.hpp"
#include "convert_impl.hpp"

namespace SafeAny
{
// Rational: since type erased numbers will always use at least 8 bytes
// it is faster to cast everything to either double, uint64_t or int64_t.
class Any
{
    template <typename T>
    using EnableIntegral =
        typename std::enable_if<std::is_integral<T>::value || std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableNonIntegral =
        typename std::enable_if<!std::is_integral<T>::value && !std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableString = typename std::enable_if<std::is_same<T, std::string>::value>::type*;

    template <typename T>
    using EnableArithmetic = typename std::enable_if<std::is_arithmetic<T>::value>::type*;

    template <typename T>
    using EnableEnum = typename std::enable_if<std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableUnknownType =
        typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_enum<T>::value &&
                                !std::is_same<T, std::string>::value>::type*;

  public:
    Any()
    {
    }

    ~Any() = default;

    Any(const double& value) : _any(value)
    {
    }

    Any(const uint64_t& value) : _any(value)
    {
    }

    Any(const float& value) : _any(double(value))
    {
    }

    Any(const std::string& str) : _any(SimpleString(str))
    {
    }

    // all the other integrals are casted to int64_t
    template <typename T>
    explicit Any(const T& value, EnableIntegral<T> = 0) : _any(int64_t(value))
    {
    }

    // default for other custom types
    template <typename T>
    explicit Any(const T& value, EnableNonIntegral<T> = 0) : _any(value)
    {
    }

    // this is different from any_cast, because if allows safe
    // conversions between arithmetic values.
    template <typename T>
    T cast() const
    {
        if (_any.type() == typeid(T))
        {
            return linb::any_cast<T>(_any);
        }
        else
        {
            return convert<T>();
        }
    }

    const std::type_info& type() const noexcept
    {
        return _any.type();
    }

  private:
    linb::any _any;

    //----------------------------

    template <typename DST>
    DST convert(EnableString<DST> = 0) const
    {
        const auto& type = _any.type();

        if (type == typeid(SimpleString))
        {
            return linb::any_cast<SimpleString>(_any).toStdString();
        }
        else if (type == typeid(int64_t))
        {
            return std::to_string(linb::any_cast<int64_t>(_any));
        }
        else if (type == typeid(uint64_t))
        {
            return std::to_string(linb::any_cast<uint64_t>(_any));
        }
        else if (type == typeid(double))
        {
            return std::to_string(linb::any_cast<double>(_any));
        }

        throw errorMsg<DST>();
    }

    template <typename DST>
    DST convert(EnableArithmetic<DST> = 0) const
    {
        using details::convertNumber;
        DST out;

        const auto& type = _any.type();

        if (type == typeid(int64_t))
        {
            convertNumber<int64_t, DST>(linb::any_cast<int64_t>(_any), out);
        }
        else if (type == typeid(uint64_t))
        {
            convertNumber<uint64_t, DST>(linb::any_cast<uint64_t>(_any), out);
        }
        else if (type == typeid(double))
        {
            convertNumber<double, DST>(linb::any_cast<double>(_any), out);
        }
        else
        {
            throw errorMsg<DST>();
        }
        return out;
    }

    template <typename DST>
    DST convert(EnableEnum<DST> = 0) const
    {
        using details::convertNumber;

        const auto& type = _any.type();

        if (type == typeid(int64_t))
        {
            uint64_t out = linb::any_cast<int64_t>(_any);
            return static_cast<DST>(out);
        }
        else if (type == typeid(uint64_t))
        {
            uint64_t out = linb::any_cast<uint64_t>(_any);
            return static_cast<DST>(out);
        }

        throw errorMsg<DST>();
    }

    template <typename DST>
    DST convert(EnableUnknownType<DST> = 0) const
    {
        throw errorMsg<DST>();
    }

    template <typename T>
    std::runtime_error errorMsg() const
    {
        char buffer[1024];
        sprintf(buffer, "[Any::convert]: no known safe conversion between %s and %s",
                _any.type().name(), typeid(T).name());
        return std::runtime_error(buffer);
    }
};

}   // end namespace VarNumber

#endif   // VARNUMBER_H
