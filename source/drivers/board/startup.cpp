#include "startup.h"


uint32_t F_CPU;


void f_cpu_init(uint32_t clock)
{
    F_CPU = clock;
}

