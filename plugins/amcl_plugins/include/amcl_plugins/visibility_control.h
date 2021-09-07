#ifndef AMCL_PLUGINS__VISIBILITY_CONTROL_H_
#define AMCL_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AMCL_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define AMCL_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define AMCL_PLUGINS_EXPORT __declspec(dllexport)
    #define AMCL_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef AMCL_PLUGINS_BUILDING_LIBRARY
    #define AMCL_PLUGINS_PUBLIC AMCL_PLUGINS_EXPORT
  #else
    #define AMCL_PLUGINS_PUBLIC AMCL_PLUGINS_IMPORT
  #endif
  #define AMCL_PLUGINS_PUBLIC_TYPE AMCL_PLUGINS_PUBLIC
  #define AMCL_PLUGINS_LOCAL
#else
  #define AMCL_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define AMCL_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define AMCL_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define AMCL_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AMCL_PLUGINS_PUBLIC
    #define AMCL_PLUGINS_LOCAL
  #endif
  #define AMCL_PLUGINS_PUBLIC_TYPE
#endif

#endif  // AMCL_PLUGINS__VISIBILITY_CONTROL_H_
