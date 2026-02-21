#ifndef CPU_DISPATCH_H
#define CPU_DISPATCH_H

#include <stdbool.h>

/* Select which CPU core to use. Must be called before cpui386_new(). */
void cpu_select_core(bool fast);

#endif /* CPU_DISPATCH_H */
