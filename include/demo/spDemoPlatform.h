
#ifndef SP_DEMO_PLATFORM_H
#define SP_DEMO_PLATFORM_H

#if (_MSC_VER)
  #define DEMO_API __declspec(dllexport)
  #define INLINE __inline
#else
  #define DEMO_API 
  #define INLINE 
#endif

#endif