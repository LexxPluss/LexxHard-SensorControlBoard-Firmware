
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_FLASH_SIMULATOR_H
#define Z_INCLUDE_SYSCALLS_FLASH_SIMULATOR_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void * z_impl_flash_simulator_get_memory(const struct device * dev, size_t * mock_size);

__pinned_func
static inline void * flash_simulator_get_memory(const struct device * dev, size_t * mock_size)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; size_t * val; } parm1 = { .val = mock_size };
		return (void *) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_FLASH_SIMULATOR_GET_MEMORY);
	}
#endif
	compiler_barrier();
	return z_impl_flash_simulator_get_memory(dev, mock_size);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
