
#ifndef SP_PLATFORM_H
#define SP_PLATFORM_H

#if (_WIN32 || _WIN64)
  #define SP_DLL __declspec(dllexport)

  #if (_MSC_VER)
    #define INLINE _inline
  #endif
#else
  #define INLINE inline
  #define SP_DLL
#endif

#endif
