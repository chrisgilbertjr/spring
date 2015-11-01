
#ifndef SP_PLATFORM_H
#define SP_PLATFORM_H

#if (_WIN32 || _WIN64)
  #if (_MSC_VER)
    #ifdef SPRING_EXPORT
      #define SPRING_API __declspec(dllexport)
    #else
      #define SPRING_API __declspec(dllimport)
    #endif

    #define INLINE __inline
  #endif
#else
  #define INLINE inline
  #define SPRING_API
#endif

#endif
