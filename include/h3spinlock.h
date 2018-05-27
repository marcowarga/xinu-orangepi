// Allwinner H3 hardware spinlock support
#include <stdint.h>

void h3_spinlock_init(void);
void h3_spinlock_acquire(uint32_t nr);
void h3_spinlock_release(uint32_t nr);
