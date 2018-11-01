//
// Copyright (c) 2014-2018 Martin Moene
//
// https://github.com/martinmoene/optional-lite
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#ifndef NONSTD_OPTIONAL_LITE_HPP
#define NONSTD_OPTIONAL_LITE_HPP

#define  optional_lite_VERSION "3.1.0"

// C++ language version detection (C++20 is speculative):
// Note: VC14.0/1900 (VS2015) lacks too much from C++14.

#ifndef   optional_CPLUSPLUS
# ifdef  _MSVC_LANG
#  define optional_CPLUSPLUS  (_MSC_VER == 1900 ? 201103L : _MSVC_LANG )
# else
#  define optional_CPLUSPLUS  __cplusplus
# endif
#endif

#define optional_CPP98_OR_GREATER  ( optional_CPLUSPLUS >= 199711L )
#define optional_CPP11_OR_GREATER  ( optional_CPLUSPLUS >= 201103L )
#define optional_CPP14_OR_GREATER  ( optional_CPLUSPLUS >= 201402L )
#define optional_CPP17_OR_GREATER  ( optional_CPLUSPLUS >= 201703L )
#define optional_CPP20_OR_GREATER  ( optional_CPLUSPLUS >= 202000L )

// C++ language version (represent 98 as 3):

#define optional_CPLUSPLUS_V  ( optional_CPLUSPLUS / 100 - (optional_CPLUSPLUS > 200000 ? 2000 : 1994) )

// use C++17 std::optional if available:

#if defined( __has_include )
# define optional_HAS_INCLUDE( arg )  __has_include( arg )
#else
# define optional_HAS_INCLUDE( arg )  0
#endif

#define optional_HAVE_STD_OPTIONAL  ( optional_CPP17_OR_GREATER && optional_HAS_INCLUDE( <optional> ) )

#if optional_HAVE_STD_OPTIONAL

#include <optional>

namespace nonstd {

    using std::optional;
    using std::bad_optional_access;
    using std::hash;

    using std::nullopt;
    using std::nullopt_t;
    using std::in_place;
    using std::in_place_type;
    using std::in_place_index;
    using std::in_place_t;
    using std::in_place_type_t;
    using std::in_place_index_t;

    using std::operator==;
    using std::operator!=;
    using std::operator<;
    using std::operator<=;
    using std::operator>;
    using std::operator>=;
    using std::make_optional;
    using std::swap;
}

#else // C++17 std::optional

#include <cassert>
#include <stdexcept>
#include <utility>

// optional-lite alignment configuration:

#ifndef  optional_CONFIG_MAX_ALIGN_HACK
# define optional_CONFIG_MAX_ALIGN_HACK  0
#endif

#ifndef  optional_CONFIG_ALIGN_AS
// no default, used in #if defined()
#endif

#ifndef  optional_CONFIG_ALIGN_AS_FALLBACK
# define optional_CONFIG_ALIGN_AS_FALLBACK  double
#endif

// Compiler warning suppression:

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wundef"
#elif defined __GNUC__
# pragma GCC   diagnostic push
# pragma GCC   diagnostic ignored "-Wundef"
#endif

// half-open range [lo..hi):
#define optional_BETWEEN( v, lo, hi ) ( lo <= v && v < hi )

#if defined(_MSC_VER) && !defined(__clang__)
# define optional_COMPILER_MSVC_VERSION   (_MSC_VER / 10 - 10 * ( 5 + (_MSC_VER < 1900)) )
#else
# define optional_COMPILER_MSVC_VERSION   0
#endif

#define optional_COMPILER_VERSION( major, minor, patch )  ( 10 * (10 * major + minor ) + patch )

#if defined __GNUC__
# define optional_COMPILER_GNUC_VERSION   optional_COMPILER_VERSION(__GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__)
#else
# define optional_COMPILER_GNUC_VERSION   0
#endif

#if defined __clang__
# define optional_COMPILER_CLANG_VERSION  optional_COMPILER_VERSION(__clang_major__, __clang_minor__, __clang_patchlevel__)
#else
# define optional_COMPILER_CLANG_VERSION  0
#endif

#if optional_BETWEEN(optional_COMPILER_MSVC_VERSION, 70, 140 )
# pragma warning( push )
# pragma warning( disable: 4345 )   // initialization behavior changed
#endif

#if optional_BETWEEN(optional_COMPILER_MSVC_VERSION, 70, 150 )
# pragma warning( push )
# pragma warning( disable: 4814 )   // in C++14 'constexpr' will not imply 'const'
#endif

// Presence of language and library features:

#define optional_HAVE(FEATURE) ( optional_HAVE_##FEATURE )

// Presence of C++11 language features:

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 100
# define optional_HAVE_AUTO  1
# define optional_HAVE_NULLPTR  1
# define optional_HAVE_STATIC_ASSERT  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 120
# define optional_HAVE_DEFAULT_FUNCTION_TEMPLATE_ARG  1
# define optional_HAVE_INITIALIZER_LIST  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 140
# define optional_HAVE_ALIAS_TEMPLATE  1
# define optional_HAVE_CONSTEXPR_11  1
# define optional_HAVE_ENUM_CLASS  1
# define optional_HAVE_EXPLICIT_CONVERSION  1
# define optional_HAVE_IS_DEFAULT  1
# define optional_HAVE_IS_DELETE  1
# define optional_HAVE_NOEXCEPT  1
# define optional_HAVE_REF_QUALIFIER  1
#endif

// Presence of C++14 language features:

#if optional_CPP14_OR_GREATER
# define optional_HAVE_CONSTEXPR_14  1
#endif

// Presence of C++17 language features:

#if optional_CPP17_OR_GREATER
# define optional_HAVE_ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE  1
#endif

// Presence of C++ library features:

#if optional_COMPILER_GNUC_VERSION
# define optional_HAVE_TR1_TYPE_TRAITS  1
# define optional_HAVE_TR1_ADD_POINTER  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 90
# define optional_HAVE_TYPE_TRAITS  1
# define optional_HAVE_STD_ADD_POINTER  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 110
# define optional_HAVE_ARRAY  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 120
# define optional_HAVE_CONDITIONAL  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 140 || (optional_COMPILER_MSVC_VERSION >= 90 && _HAS_CPP0X)
# define optional_HAVE_CONTAINER_DATA_METHOD  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 120
# define optional_HAVE_REMOVE_CV  1
#endif

#if optional_CPP11_OR_GREATER || optional_COMPILER_MSVC_VERSION >= 140
# define optional_HAVE_SIZED_TYPES  1
#endif

// For the rest, consider VC14 as C++11 for optional-lite:

#if optional_COMPILER_MSVC_VERSION >= 140
# undef  optional_CPP11_OR_GREATER
# define optional_CPP11_OR_GREATER  1
#endif

// C++ feature usage:

#if optional_HAVE( CONSTEXPR_11 )
# define optional_constexpr  constexpr
#else
# define optional_constexpr  /*constexpr*/
#endif

#if optional_HAVE( CONSTEXPR_14 )
# define optional_constexpr14  constexpr
#else
# define optional_constexpr14  /*constexpr*/
#endif

#if optional_HAVE( NOEXCEPT )
# define optional_noexcept  noexcept
#else
# define optional_noexcept  /*noexcept*/
#endif

#if optional_HAVE( NULLPTR )
# define optional_nullptr  nullptr
#else
# define optional_nullptr  NULL
#endif

#if optional_HAVE( REF_QUALIFIER )
# define optional_ref_qual  &
# define optional_refref_qual  &&
#else
# define optional_ref_qual  /*&*/
# define optional_refref_qual  /*&&*/
#endif

// additional includes:

#if optional_CPP11_OR_GREATER
# include <functional>
#endif

#if optional_HAVE( INITIALIZER_LIST )
# include <initializer_list>
#endif

#if optional_HAVE( TYPE_TRAITS )
# include <type_traits>
#elif optional_HAVE( TR1_TYPE_TRAITS )
# include <tr1/type_traits>
#endif

// type traits needed:

namespace BT { namespace optional_lite { namespace detail {

#if optional_HAVE( CONDITIONAL )
    using std::conditional;
#else
    template< bool B, typename T, typename F > struct conditional              { typedef T type; };
    template<         typename T, typename F > struct conditional<false, T, F> { typedef F type; };
#endif // optional_HAVE_CONDITIONAL

}}}

//
// in_place: code duplicated in any-lite, optional-lite, variant-lite:
//

#ifndef nonstd_lite_HAVE_IN_PLACE_TYPES

namespace BT {

namespace detail {

template< class T >
struct in_place_type_tag {};

template< std::size_t I >
struct in_place_index_tag {};

} // namespace detail

struct in_place_t {};

template< class T >
inline in_place_t in_place( detail::in_place_type_tag<T> = detail::in_place_type_tag<T>() )
{
    return in_place_t();
}

template< std::size_t I >
inline in_place_t in_place( detail::in_place_index_tag<I> = detail::in_place_index_tag<I>() )
{
    return in_place_t();
}

template< class T >
inline in_place_t in_place_type( detail::in_place_type_tag<T> = detail::in_place_type_tag<T>() )
{
    return in_place_t();
}

template< std::size_t I >
inline in_place_t in_place_index( detail::in_place_index_tag<I> = detail::in_place_index_tag<I>() )
{
    return in_place_t();
}

// mimic templated typedef:

#define nonstd_lite_in_place_type_t( T)  BT::in_place_t(&)( BT::detail::in_place_type_tag<T>  )
#define nonstd_lite_in_place_index_t(T)  BT::in_place_t(&)( BT::detail::in_place_index_tag<I> )

#define nonstd_lite_HAVE_IN_PLACE_TYPES  1

} // namespace nonstd

#endif // nonstd_lite_HAVE_IN_PLACE_TYPES

//
// optional:
//

namespace BT { namespace optional_lite {

/// class optional

template< typename T >
class optional;

namespace detail {

// C++11 emulation:

struct nulltype{};

template< typename Head, typename Tail >
struct typelist
{
    typedef Head head;
    typedef Tail tail;
};

#if optional_CONFIG_MAX_ALIGN_HACK

// Max align, use most restricted type for alignment:

#define optional_UNIQUE(  name )       optional_UNIQUE2( name, __LINE__ )
#define optional_UNIQUE2( name, line ) optional_UNIQUE3( name, line )
#define optional_UNIQUE3( name, line ) name ## line

#define optional_ALIGN_TYPE( type ) \
    type optional_UNIQUE( _t ); struct_t< type > optional_UNIQUE( _st )

template< typename T >
struct struct_t { T _; };

union max_align_t
{
    optional_ALIGN_TYPE( char );
    optional_ALIGN_TYPE( short int );
    optional_ALIGN_TYPE( int );
    optional_ALIGN_TYPE( long int  );
    optional_ALIGN_TYPE( float  );
    optional_ALIGN_TYPE( double );
    optional_ALIGN_TYPE( long double );
    optional_ALIGN_TYPE( char * );
    optional_ALIGN_TYPE( short int * );
    optional_ALIGN_TYPE( int *  );
    optional_ALIGN_TYPE( long int * );
    optional_ALIGN_TYPE( float * );
    optional_ALIGN_TYPE( double * );
    optional_ALIGN_TYPE( long double * );
    optional_ALIGN_TYPE( void * );

#ifdef HAVE_LONG_LONG
    optional_ALIGN_TYPE( long long );
#endif

    struct Unknown;

    Unknown ( * optional_UNIQUE(_) )( Unknown );
    Unknown * Unknown::* optional_UNIQUE(_);
    Unknown ( Unknown::* optional_UNIQUE(_) )( Unknown );

    struct_t< Unknown ( * )( Unknown)         > optional_UNIQUE(_);
    struct_t< Unknown * Unknown::*            > optional_UNIQUE(_);
    struct_t< Unknown ( Unknown::* )(Unknown) > optional_UNIQUE(_);
};

#undef optional_UNIQUE
#undef optional_UNIQUE2
#undef optional_UNIQUE3

#undef optional_ALIGN_TYPE

#elif defined( optional_CONFIG_ALIGN_AS ) // optional_CONFIG_MAX_ALIGN_HACK

// Use user-specified type for alignment:

#define optional_ALIGN_AS( unused ) \
    optional_CONFIG_ALIGN_AS

#else // optional_CONFIG_MAX_ALIGN_HACK

// Determine POD type to use for alignment:

#define optional_ALIGN_AS( to_align ) \
    typename type_of_size< alignment_types, alignment_of< to_align >::value >::type

template <typename T>
struct alignment_of;

template <typename T>
struct alignment_of_hack
{
    char c;
    T t;
    alignment_of_hack();
};

template <unsigned A, unsigned S>
struct alignment_logic
{
    enum { value = A < S ? A : S };
};

template< typename T >
struct alignment_of
{
    enum { value = alignment_logic<
        sizeof( alignment_of_hack<T> ) - sizeof(T), sizeof(T) >::value, };
};

template< typename List, size_t N >
struct type_of_size
{
    typedef typename conditional<
        N == sizeof( typename List::head ),
            typename List::head,
            typename type_of_size<typename List::tail, N >::type >::type type;
};

template< size_t N >
struct type_of_size< nulltype, N >
{
    typedef optional_CONFIG_ALIGN_AS_FALLBACK type;
};

template< typename T>
struct struct_t { T _; };

#define optional_ALIGN_TYPE( type ) \
    typelist< type , typelist< struct_t< type >

struct Unknown;

typedef
    optional_ALIGN_TYPE( char ),
    optional_ALIGN_TYPE( short ),
    optional_ALIGN_TYPE( int ),
    optional_ALIGN_TYPE( long ),
    optional_ALIGN_TYPE( float ),
    optional_ALIGN_TYPE( double ),
    optional_ALIGN_TYPE( long double ),

    optional_ALIGN_TYPE( char *),
    optional_ALIGN_TYPE( short * ),
    optional_ALIGN_TYPE( int * ),
    optional_ALIGN_TYPE( long * ),
    optional_ALIGN_TYPE( float * ),
    optional_ALIGN_TYPE( double * ),
    optional_ALIGN_TYPE( long double * ),

    optional_ALIGN_TYPE( Unknown ( * )( Unknown ) ),
    optional_ALIGN_TYPE( Unknown * Unknown::*     ),
    optional_ALIGN_TYPE( Unknown ( Unknown::* )( Unknown ) ),

    nulltype
    > > > > > > >    > > > > > > >
    > > > > > > >    > > > > > > >
    > > > > > >
    alignment_types;

#undef optional_ALIGN_TYPE

#endif // optional_CONFIG_MAX_ALIGN_HACK

/// C++03 constructed union to hold value.

template< typename T >
union storage_t
{
private:
    friend class optional<T>;

    typedef T value_type;

    storage_t() {}

    storage_t( value_type const & v )
    {
        construct_value( v );
    }

    void construct_value( value_type const & v )
    {
        ::new( value_ptr() ) value_type( v );
    }

#if optional_CPP11_OR_GREATER

    storage_t( value_type && v )
    {
        construct_value( std::move( v ) );
    }

    void construct_value( value_type && v )
    {
        ::new( value_ptr() ) value_type( std::move( v ) );
    }

    template< class... Args >
    void emplace( Args&&... args )
    {
        ::new( value_ptr() ) value_type( std::forward<Args>(args)... );
    }

    template< class U, class... Args >
    void emplace( std::initializer_list<U> il, Args&&... args )
    {
        ::new( value_ptr() ) value_type( il, std::forward<Args>(args)... );
    }

#endif

    void destruct_value()
    {
        value_ptr()->~T();
    }

    value_type const * value_ptr() const
    {
        return as<value_type>();
    }

    value_type * value_ptr()
    {
        return as<value_type>();
    }

    value_type const & value() const optional_ref_qual
    {
        return * value_ptr();
    }

    value_type & value() optional_ref_qual
    {
        return * value_ptr();
    }

#if optional_CPP11_OR_GREATER

    value_type const && value() const optional_refref_qual
    {
        return std::move( value() );
    }

    value_type && value() optional_refref_qual
    {
        return std::move( value() );
    }

#endif

#if optional_CPP11_OR_GREATER

    using aligned_storage_t = typename std::aligned_storage< sizeof(value_type), alignof(value_type) >::type;
    aligned_storage_t data;

#elif optional_CONFIG_MAX_ALIGN_HACK

    typedef struct { unsigned char data[ sizeof(value_type) ]; } aligned_storage_t;

    max_align_t hack;
    aligned_storage_t data;

#else
    typedef optional_ALIGN_AS(value_type) align_as_type;

    typedef struct { align_as_type data[ 1 + ( sizeof(value_type) - 1 ) / sizeof(align_as_type) ]; } aligned_storage_t;
    aligned_storage_t data;

#   undef optional_ALIGN_AS

#endif // optional_CONFIG_MAX_ALIGN_HACK

    void * ptr() optional_noexcept
    {
        return &data;
    }

    void const * ptr() const optional_noexcept
    {
        return &data;
    }

    template <typename U>
    U * as()
    {
        return reinterpret_cast<U*>( ptr() );
    }

    template <typename U>
    U const * as() const
    {
        return reinterpret_cast<U const *>( ptr() );
    }
};

} // namespace detail

/// disengaged state tag

struct nullopt_t
{
    struct init{};
    optional_constexpr nullopt_t( init ) {}
};

#if optional_HAVE( CONSTEXPR_11 )
constexpr nullopt_t nullopt{ nullopt_t::init{} };
#else
// extra parenthesis to prevent the most vexing parse:
const nullopt_t nullopt(( nullopt_t::init() ));
#endif

/// optional access error

class bad_optional_access : public std::logic_error
{
public:
  explicit bad_optional_access()
  : logic_error( "bad optional access" ) {}
};

/// optional

template< typename T>
class optional
{
private:
    typedef void (optional::*safe_bool)() const;

public:
    typedef T value_type;

    optional_constexpr optional() optional_noexcept
    : has_value_( false )
    , contained()
    {}

    optional_constexpr optional( nullopt_t ) optional_noexcept
    : has_value_( false )
    , contained()
    {}

    optional( optional const & rhs )
    : has_value_( rhs.has_value() )
    {
        if ( rhs.has_value() )
            contained.construct_value( rhs.contained.value() );
    }

#if optional_CPP11_OR_GREATER
    optional_constexpr14 optional( optional && rhs ) noexcept( std::is_nothrow_move_constructible<T>::value )
    : has_value_( rhs.has_value() )
    {
        if ( rhs.has_value() )
            contained.construct_value( std::move( rhs.contained.value() ) );
    }
#endif

    optional_constexpr optional( value_type const & value )
    : has_value_( true )
    , contained( value )
    {}

#if optional_CPP11_OR_GREATER

    optional_constexpr optional( value_type && value )
    : has_value_( true )
    , contained( std::move( value ) )
    {}

    template< class... Args >
    optional_constexpr explicit optional( nonstd_lite_in_place_type_t(T), Args&&... args )
    : has_value_( true )
    , contained( T( std::forward<Args>(args)...) )
    {}

    template< class U, class... Args >
    optional_constexpr explicit optional( nonstd_lite_in_place_type_t(T), std::initializer_list<U> il, Args&&... args )
    : has_value_( true )
    , contained( T( il, std::forward<Args>(args)...) )
    {}

#endif // optional_CPP11_OR_GREATER

    ~optional()
    {
        if ( has_value() )
            contained.destruct_value();
    }

    // assignment

    optional & operator=( nullopt_t ) optional_noexcept
    {
        reset();
        return *this;
    }

    optional & operator=( optional const & rhs )
#if optional_CPP11_OR_GREATER
        noexcept( std::is_nothrow_move_assignable<T>::value && std::is_nothrow_move_constructible<T>::value )
#endif
    {
        if      ( has_value() == true  && rhs.has_value() == false ) reset();
        else if ( has_value() == false && rhs.has_value() == true  ) initialize( *rhs );
        else if ( has_value() == true  && rhs.has_value() == true  ) contained.value() = *rhs;
        return *this;
    }

#if optional_CPP11_OR_GREATER

    optional & operator=( optional && rhs ) noexcept
    {
        if      ( has_value() == true  && rhs.has_value() == false ) reset();
        else if ( has_value() == false && rhs.has_value() == true  ) initialize( std::move( *rhs ) );
        else if ( has_value() == true  && rhs.has_value() == true  ) contained.value() = std::move( *rhs );
        return *this;
    }

    template< class U,
        typename = typename std::enable_if< std::is_same< typename std::decay<U>::type, T>::value >::type >
    optional & operator=( U && v )
    {
        if ( has_value() ) contained.value() = std::forward<U>( v );
        else               initialize( T( std::forward<U>( v ) ) );
        return *this;
    }

    template< class... Args >
    void emplace( Args&&... args )
    {
        *this = nullopt;
        contained.emplace( std::forward<Args>(args)...  );
        has_value_ = true;
    }


    template< class U, class... Args >
    void emplace( std::initializer_list<U> il, Args&&... args )
    {
        *this = nullopt;
        contained.emplace( il, std::forward<Args>(args)...  );
        has_value_ = true;
    }

#endif // optional_CPP11_OR_GREATER

    // swap

    void swap( optional & rhs )
#if optional_CPP11_OR_GREATER
    noexcept( std::is_nothrow_move_constructible<T>::value && noexcept( std::swap( std::declval<T&>(), std::declval<T&>() ) ) )
#endif
    {
        using std::swap;
        if      ( has_value() == true  && rhs.has_value() == true  ) { swap( **this, *rhs ); }
        else if ( has_value() == false && rhs.has_value() == true  ) { initialize( *rhs ); rhs.reset(); }
        else if ( has_value() == true  && rhs.has_value() == false ) { rhs.initialize( **this ); reset(); }
    }

    // observers

    optional_constexpr value_type const * operator ->() const
    {
        return assert( has_value() ),
            contained.value_ptr();
    }

    optional_constexpr14 value_type * operator ->()
    {
        return assert( has_value() ),
            contained.value_ptr();
    }

    optional_constexpr value_type const & operator *() const optional_ref_qual
    {
        return assert( has_value() ),
            contained.value();
    }

    optional_constexpr14 value_type & operator *() optional_ref_qual
    {
        return assert( has_value() ),
            contained.value();
    }

#if optional_CPP11_OR_GREATER

    optional_constexpr value_type const && operator *() const optional_refref_qual
    {
        return std::move( **this );
    }

    optional_constexpr14 value_type && operator *() optional_refref_qual
    {
        return std::move( **this );
    }

#endif

#if optional_CPP11_OR_GREATER
    optional_constexpr explicit operator bool() const optional_noexcept
    {
        return has_value();
    }
#else
    optional_constexpr operator safe_bool() const optional_noexcept
    {
        return has_value() ? &optional::this_type_does_not_support_comparisons : 0;
    }
#endif

    optional_constexpr bool has_value() const optional_noexcept
    {
        return has_value_;
    }

    optional_constexpr14 value_type const & value() const optional_ref_qual
    {
        if ( ! has_value() )
            throw bad_optional_access();

        return contained.value();
    }

    optional_constexpr14 value_type & value() optional_ref_qual
    {
        if ( ! has_value() )
            throw bad_optional_access();

        return contained.value();
    }

#if optional_HAVE( REF_QUALIFIER )

    optional_constexpr14 value_type const && value() const optional_refref_qual
    {
        return std::move( value() );
    }

    optional_constexpr14 value_type && value() optional_refref_qual
    {
        return std::move( value() );
    }

#endif

#if optional_CPP11_OR_GREATER

    template< class U >
    optional_constexpr value_type value_or( U && v ) const optional_ref_qual
    {
        return has_value() ? contained.value() : static_cast<T>(std::forward<U>( v ) );
    }

    template< class U >
    optional_constexpr value_type value_or( U && v ) const optional_refref_qual
    {
        return has_value() ? std::move( contained.value() ) : static_cast<T>(std::forward<U>( v ) );
    }

#else

    template< class U >
    optional_constexpr value_type value_or( U const & v ) const
    {
        return has_value() ? contained.value() : static_cast<value_type>( v );
    }

#endif // optional_CPP11_OR_GREATER

    // modifiers

    void reset() optional_noexcept
    {
        if ( has_value() )
            contained.destruct_value();

        has_value_ = false;
    }

private:
    void this_type_does_not_support_comparisons() const {}

    template< typename V >
    void initialize( V const & value )
    {
        assert( ! has_value()  );
        contained.construct_value( value );
        has_value_ = true;
    }

#if optional_CPP11_OR_GREATER
    template< typename V >
    void initialize( V && value )
    {
        assert( ! has_value()  );
        contained.construct_value( std::move( value ) );
        has_value_ = true;
    }

#endif

private:
    bool has_value_;
    detail::storage_t< value_type > contained;

};

// Relational operators

template< typename T, typename U >
inline optional_constexpr bool operator==( optional<T> const & x, optional<U> const & y )
{
    return bool(x) != bool(y) ? false : !bool( x ) ? true : *x == *y;
}

template< typename T, typename U >
inline optional_constexpr bool operator!=( optional<T> const & x, optional<U> const & y )
{
    return !(x == y);
}

template< typename T, typename U >
inline optional_constexpr bool operator<( optional<T> const & x, optional<U> const & y )
{
    return (!y) ? false : (!x) ? true : *x < *y;
}

template< typename T, typename U >
inline optional_constexpr bool operator>( optional<T> const & x, optional<U> const & y )
{
    return (y < x);
}

template< typename T, typename U >
inline optional_constexpr bool operator<=( optional<T> const & x, optional<U> const & y )
{
    return !(y < x);
}

template< typename T, typename U >
inline optional_constexpr bool operator>=( optional<T> const & x, optional<U> const & y )
{
    return !(x < y);
}

// Comparison with nullopt

template< typename T >
inline optional_constexpr bool operator==( optional<T> const & x, nullopt_t ) optional_noexcept
{
    return (!x);
}

template< typename T >
inline optional_constexpr bool operator==( nullopt_t, optional<T> const & x ) optional_noexcept
{
    return (!x);
}

template< typename T >
inline optional_constexpr bool operator!=( optional<T> const & x, nullopt_t ) optional_noexcept
{
    return bool(x);
}

template< typename T >
inline optional_constexpr bool operator!=( nullopt_t, optional<T> const & x ) optional_noexcept
{
    return bool(x);
}

template< typename T >
inline optional_constexpr bool operator<( optional<T> const &, nullopt_t ) optional_noexcept
{
    return false;
}

template< typename T >
inline optional_constexpr bool operator<( nullopt_t, optional<T> const & x ) optional_noexcept
{
    return bool(x);
}

template< typename T >
inline optional_constexpr bool operator<=( optional<T> const & x, nullopt_t ) optional_noexcept
{
    return (!x);
}

template< typename T >
inline optional_constexpr bool operator<=( nullopt_t, optional<T> const & ) optional_noexcept
{
    return true;
}

template< typename T >
inline optional_constexpr bool operator>( optional<T> const & x, nullopt_t ) optional_noexcept
{
    return bool(x);
}

template< typename T >
inline optional_constexpr bool operator>( nullopt_t, optional<T> const & ) optional_noexcept
{
    return false;
}

template< typename T >
inline optional_constexpr bool operator>=( optional<T> const &, nullopt_t ) optional_noexcept
{
    return true;
}

template< typename T >
inline optional_constexpr bool operator>=( nullopt_t, optional<T> const & x ) optional_noexcept
{
    return (!x);
}

// Comparison with T

template< typename T, typename U >
inline optional_constexpr bool operator==( optional<T> const & x, U const & v )
{
    return bool(x) ? *x == v : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator==( U const & v, optional<T> const & x )
{
    return bool(x) ? v == *x : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator!=( optional<T> const & x, U const & v )
{
    return bool(x) ? *x != v : true;
}

template< typename T, typename U >
inline optional_constexpr bool operator!=( U const & v, optional<T> const & x )
{
    return bool(x) ? v != *x : true;
}

template< typename T, typename U >
inline optional_constexpr bool operator<( optional<T> const & x, U const & v )
{
    return bool(x) ? *x < v : true;
}

template< typename T, typename U >
inline optional_constexpr bool operator<( U const & v, optional<T> const & x )
{
    return bool(x) ? v < *x : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator<=( optional<T> const & x, U const & v )
{
    return bool(x) ? *x <= v : true;
}

template< typename T, typename U >
inline optional_constexpr bool operator<=( U const & v, optional<T> const & x )
{
    return bool(x) ? v <= *x : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator>( optional<T> const & x, U const & v )
{
    return bool(x) ? *x > v : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator>( U const & v, optional<T> const & x )
{
    return bool(x) ? v > *x : true;
}

template< typename T, typename U >
inline optional_constexpr bool operator>=( optional<T> const & x, U const & v )
{
    return bool(x) ? *x >= v : false;
}

template< typename T, typename U >
inline optional_constexpr bool operator>=( U const & v, optional<T> const & x )
{
    return bool(x) ? v >= *x : true;
}

// Specialized algorithms

template< typename T >
void swap( optional<T> & x, optional<T> & y )
#if optional_CPP11_OR_GREATER
    noexcept( noexcept( x.swap(y) ) )
#endif
{
    x.swap( y );
}

#if optional_CPP11_OR_GREATER

template< class T >
optional_constexpr optional< typename std::decay<T>::type > make_optional( T && v )
{
    return optional< typename std::decay<T>::type >( std::forward<T>( v ) );
}

template< class T, class...Args >
optional_constexpr optional<T> make_optional( Args&&... args )
{
    return optional<T>( in_place, std::forward<Args>(args)...);
}

template< class T, class U, class... Args >
optional_constexpr optional<T> make_optional( std::initializer_list<U> il, Args&&... args )
{
    return optional<T>( in_place, il, std::forward<Args>(args)...);
}

#else

template< typename T >
optional<T> make_optional( T const & v )
{
    return optional<T>( v );
}

#endif // optional_CPP11_OR_GREATER

} // namespace optional

using namespace optional_lite;

} // namespace nonstd

#if optional_CPP11_OR_GREATER

// specialize the std::hash algorithm:

namespace std {

template< class T >
struct hash< BT::optional<T> >
{
public:
    std::size_t operator()( BT::optional<T> const & v ) const optional_noexcept
    {
        return bool( v ) ? hash<T>()( *v ) : 0;
    }
};

} //namespace std

#endif // optional_CPP11_OR_GREATER

#ifdef __clang__
# pragma clang diagnostic pop
#elif defined __GNUC__
# pragma GCC   diagnostic pop
#endif

#endif // have C++17 std::optional

#endif // NONSTD_OPTIONAL_LITE_HPP
