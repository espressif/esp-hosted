#include "../../unit/arch/cc.h"

#ifndef LWIP_RAND
#define LWIP_RAND() ((u32_t)rand())
#endif
