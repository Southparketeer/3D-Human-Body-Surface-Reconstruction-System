#ifndef KANGAROO_CONFIG_H
#define KANGAROO_CONFIG_H

/// Platform
/* #undef _UNIX_ */
#define _WIN_
/* #undef _OSX_ */
/* #undef _LINUX_ */

/// Compiler
/* #undef _GCC_ */
/* #undef _CLANG_ */
#define _MSVC_

/// Configured libraries
/* #undef HAVE_EIGEN */
/* #undef HAVE_ASSIMP */
#define HAVE_THRUST
#define HAVE_NPP
/* #undef HAVE_OPENCV */

/// CUDA Toolkit Version
#define CUDA_VERSION_MAJOR 7
#define CUDA_VERSION_MINOR 5

/// Defines generated when calling into Kangaroo API. Not to be
/// used in compiled library code, only inlined header code.
#if (__cplusplus > 199711L) || (_MSC_VER >= 1700)
#define CALLEE_HAS_CPP11
#define CALLEE_HAS_RVALREF
#endif

#endif // KANGAROO_CONFIG_H
