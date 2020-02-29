// clang-format off
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COMPOSITION_EXPORT __attribute__ ((dllexport))
    #define COMPOSITION_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPOSITION_EXPORT __declspec(dllexport)
    #define COMPOSITION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COMPOSITION_BUILDING_DLL
    #define ACROBAT_COMPOSITION_PUBLIC COMPOSITION_EXPORT
  #else
    #define ACROBAT_COMPOSITION_PUBLIC COMPOSITION_IMPORT
  #endif
  #define COMPOSITION_PUBLIC_TYPE ACROBAT_COMPOSITION_PUBLIC
  #define ACROBAT_COMPOSITION_LOCAL
#else
  #define COMPOSITION_EXPORT __attribute__ ((visibility("default")))
  #define COMPOSITION_IMPORT
  #if __GNUC__ >= 4
    #define ACROBAT_COMPOSITION_PUBLIC __attribute__ ((visibility("default")))
    #define ACROBAT_COMPOSITION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACROBAT_COMPOSITION_PUBLIC
    #define ACROBAT_COMPOSITION_LOCAL
  #endif
  #define COMPOSITION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // COMPOSITION__VISIBILITY_CONTROL_H_