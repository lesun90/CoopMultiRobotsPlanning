#ifndef MP_DEFINITIONS_HPP_
#define MP_DEFINITIONS_HPP_


/**
 *@cond
 */
    
/**
 *@author Erion Plaku
 *@brief Recognize operating system: Linux 
 */
#if defined (linux) || defined (__linux__)
#define OS_LINUX
#endif
    
/**
 *@author Erion Plaku
 *@brief Recognize operating system: Microsoft Windows
 */
#if defined (WINDOWS) || defined (_WIN32) || defined (__WIN32__)
#define OS_WINDOWS 
#endif
    
/**
 *@author Erion Plaku
 *@brief Recognize operating system: Macintosh OS
 */
#ifdef __APPLE__
#define OS_MAC
#endif
    
/**
 *@author Erion Plaku
 *@brief Recognize operating system: SunOS/Solaris
 */
#if defined(sun) || defined(__sun)
#define OS_SUNOS
#endif
    
/**
 *@author <a href="http:/www.cs.jhu.edu/~erion">Erion Plaku</a>
 *@brief Recognize compiler: GNU C/C++
 */
#ifdef __GNUC__
#define COMPILER_GNU
#endif
    
/**
 *@author Erion Plaku
 *@brief Recognize compiler: PGI
 */
#ifdef __PGI
#define COMPILER_PGI
#endif
    
/**
 *@author Erion Plaku
 *@brief Recognize compiler: Microsoft Visual Studio 
 */
#ifdef _MSC_VER
#define COMPILER_VISUAL_STUDIO
#endif
    
    
/**
 *@author Erion Plaku
 *@brief Make Visual Studio happy: avoid warnings/errors about
 *       <em>strdup</em> and <em>inline</em> 
 */
#ifdef COMPILER_VISUAL_STUDIO
#define _USE_MATH_DEFINES
#define strdup _strdup
#define inline __inline
#include <cstdlib>
#include <cmath>
static inline int round(const double x) {return floor(x + 0.5);}

#ifdef _MSC_VER >= 1400
  #define _CRT_SECURE_NO_DEPRECATE
  #define COMPILER_VISUAL_STUDIO_2005_OR_LATER
#else
  #define NO_VARIADIC_MACROS
#endif

#endif
    
/**
 *@author Erion Plaku
 *@brief Namespace STL extension: different compilers place
 *       STL extensions in different directories and use different
 *       naming conventions for the file names 
 */
#ifdef COMPILER_PGI
#define STD_EXT_NAMESPACE _STLP_STD
#define STD_EXT_INCLUDE(fname) <fname>
#elif defined COMPILER_GNU
#define STD_EXT_NAMESPACE __gnu_cxx
#define STD_EXT_INCLUDE(fname) <ext/fname>
#else
#define STD_EXT_NAMESPACE stdext
#define STD_EXT_INCLUDE(fname) <fname>
#endif
    
/**
 *@endcond
 */

#if DEBUG
#include <cassert>
#endif

typedef unsigned long long MyKey;



#endif
