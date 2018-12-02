#include <iostream>
#include <cstring>
#ifdef NDEBUG
#define TRACE(f)
#else
#define __FILENAME__ ((strrchr(__FILE__, '/')) ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define TRACE(f) std::cout << __FILENAME__ << ", Ln " << __LINE__ << " (" << __FUNCTION__ << "): " << f << std::endl;
#endif