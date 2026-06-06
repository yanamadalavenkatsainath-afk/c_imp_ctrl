#ifndef TARGET_PORT_H
#define TARGET_PORT_H

#ifdef TARGET_BUILD
  #define GNC_LOG(...) do { } while (0)
#else
  #include <stdio.h>
  #define GNC_LOG(...) printf(__VA_ARGS__)
#endif

#endif /* TARGET_PORT_H */
