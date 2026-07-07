#ifndef DROPPER_ACTION_CPP__VISIBILITY_CONTROL_H_
#define DROPPER_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DROPPER_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define DROPPER_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define DROPPER_ACTION_CPP_EXPORT __declspec(dllexport)
    #define DROPPER_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef DROPPER_ACTION_CPP_BUILDING_DLL
    #define DROPPER_ACTION_CPP_PUBLIC DROPPER_ACTION_CPP_EXPORT
  #else
    #define DROPPER_ACTION_CPP_PUBLIC DROPPER_ACTION_CPP_IMPORT
  #endif
  #define DROPPER_ACTION_CPP_PUBLIC_TYPE DROPPER_ACTION_CPP_PUBLIC
  #define DROPPER_ACTION_CPP_LOCAL
#else
  #define DROPPER_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define DROPPER_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define DROPPER_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define DROPPER_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DROPPER_ACTION_CPP_PUBLIC
    #define DROPPER_ACTION_CPP_LOCAL
  #endif
  #define DROPPER_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DROPPER_ACTION_CPP__VISIBILITY_CONTROL_H_