
#if (_WIN32 || _WIN64)
  #if (_MSC_VER  >= 1600)
    #define INLINE _inline
  #endif
#else
#define INLINE inline
#endif
