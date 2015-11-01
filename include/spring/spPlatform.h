
#ifndef SP_PLATFORM_H
#define SP_PLATFORM_H

#if (_MSC_VER)
  #define SPRING_API __declspec(dllexport)
  #define INLINE __inline
#else
  #define SPRING_API 
  #define INLINE 
#endif

#endif
