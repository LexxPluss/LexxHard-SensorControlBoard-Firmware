
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_DEVICE_H
#define Z_INCLUDE_SYSCALLS_DEVICE_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const struct device * z_impl_device_get_binding(const char * name);

__pinned_func
static inline const struct device * device_get_binding(const char * name)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const char * val; } parm0 = { .val = name };
		return (const struct device *) arch_syscall_invoke1(parm0.x, K_SYSCALL_DEVICE_GET_BINDING);
	}
#endif
	compiler_barrier();
	return z_impl_device_get_binding(name);
}


extern int z_impl_device_usable_check(const struct device * dev);

__pinned_func
static inline int device_usable_check(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_DEVICE_USABLE_CHECK);
	}
#endif
	compiler_barrier();
	return z_impl_device_usable_check(dev);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
