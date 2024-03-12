
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_PECI_H
#define Z_INCLUDE_SYSCALLS_PECI_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_peci_config(const struct device * dev, uint32_t bitrate);

__pinned_func
static inline int peci_config(const struct device * dev, uint32_t bitrate)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = bitrate };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_PECI_CONFIG);
	}
#endif
	compiler_barrier();
	return z_impl_peci_config(dev, bitrate);
}


extern int z_impl_peci_enable(const struct device * dev);

__pinned_func
static inline int peci_enable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_PECI_ENABLE);
	}
#endif
	compiler_barrier();
	return z_impl_peci_enable(dev);
}


extern int z_impl_peci_disable(const struct device * dev);

__pinned_func
static inline int peci_disable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_PECI_DISABLE);
	}
#endif
	compiler_barrier();
	return z_impl_peci_disable(dev);
}


extern int z_impl_peci_transfer(const struct device * dev, struct peci_msg * msg);

__pinned_func
static inline int peci_transfer(const struct device * dev, struct peci_msg * msg)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; struct peci_msg * val; } parm1 = { .val = msg };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_PECI_TRANSFER);
	}
#endif
	compiler_barrier();
	return z_impl_peci_transfer(dev, msg);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */