#ifndef DRONE_NODE__VISIBILITY_CONTROL_H_
#define DRONE_NODE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DRONE_NODE_EXPORT __attribute__ ((dllexport))
    #define DRONE_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define DRONE_NODE_EXPORT __declspec(dllexport)
    #define DRONE_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DRONE_NODE_BUILDING_DLL
    #define DRONE_NODE_PUBLIC DRONE_NODE_EXPORT
  #else
    #define DRONE_NODE_PUBLIC DRONE_NODE_IMPORT
  #endif
  #define DRONE_NODE_PUBLIC_TYPE DRONE_NODE_PUBLIC
  #define DRONE_NODE_LOCAL
#else
  #define DRONE_NODE_EXPORT __attribute__ ((visibility("default")))
  #define DRONE_NODE_IMPORT
  #if __GNUC__ >= 4
    #define DRONE_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define DRONE_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DRONE_NODE_PUBLIC
    #define DRONE_NODE_LOCAL
  #endif
  #define DRONE_NODE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DRONE_NODE__VISIBILITY_CONTROL_H_
