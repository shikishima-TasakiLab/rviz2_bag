#ifndef RVIZ2_BAG__VISIBILITY_CONTROL_HPP_
#define RVIZ2_BAG__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RVIZ2_BAG_EXPORT __attribute__ ((dllexport))
    #define RVIZ2_BAG_IMPORT __attribute__ ((dllimport))
  #else
    #define RVIZ2_BAG_EXPORT __declspec(dllexport)
    #define RVIZ2_BAG_IMPORT __declspec(dllimport)
  #endif
  #ifdef RVIZ2_BAG_BUILDING_LIBRARY
    #define RVIZ2_BAG_PUBLIC RVIZ2_BAG_EXPORT
  #else
    #define RVIZ2_BAG_PUBLIC RVIZ2_BAG_IMPORT
  #endif
  #define RVIZ2_BAG_PUBLIC_TYPE RVIZ2_BAG_PUBLIC
  #define RVIZ2_BAG_LOCAL
#else
  #define RVIZ2_BAG_EXPORT __attribute__ ((visibility("default")))
  #define RVIZ2_BAG_IMPORT
  #if __GNUC__ >= 4
    #define RVIZ2_BAG_PUBLIC __attribute__ ((visibility("default")))
    #define RVIZ2_BAG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RVIZ2_BAG_PUBLIC
    #define RVIZ2_BAG_LOCAL
  #endif
  #define RVIZ2_BAG_PUBLIC_TYPE
#endif

#endif  // RVIZ2_BAG__VISIBILITY_CONTROL_HPP_
