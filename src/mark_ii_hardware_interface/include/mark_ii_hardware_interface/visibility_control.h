#ifndef MARK_II_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define MARK_II_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MARK_II_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define MARK_II_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define MARK_II_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define MARK_II_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MARK_II_HARDWARE_INTERFACE_BUILDING_DLL
    #define MARK_II_HARDWARE_INTERFACE_PUBLIC MARK_II_HARDWARE_INTERFACE_EXPORT
  #else
    #define MARK_II_HARDWARE_INTERFACE_PUBLIC MARK_II_HARDWARE_INTERFACE_IMPORT
  #endif
  #define MARK_II_HARDWARE_INTERFACE_PUBLIC_TYPE MARK_II_HARDWARE_INTERFACE_PUBLIC
  #define MARK_II_HARDWARE_INTERFACE_LOCAL
#else
  #define MARK_II_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define MARK_II_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define MARK_II_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define MARK_II_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MARK_II_HARDWARE_INTERFACE_PUBLIC
    #define MARK_II_HARDWARE_INTERFACE_LOCAL
  #endif
  #define MARK_II_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // MARK_II_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
