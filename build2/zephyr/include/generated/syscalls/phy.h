
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_PHY_H
#define Z_INCLUDE_SYSCALLS_PHY_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_phy_configure_link(const struct device * dev, enum phy_link_speed speeds);

__pinned_func
static inline int phy_configure_link(const struct device * dev, enum phy_link_speed speeds)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; enum phy_link_speed val; } parm1 = { .val = speeds };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_PHY_CONFIGURE_LINK);
	}
#endif
	compiler_barrier();
	return z_impl_phy_configure_link(dev, speeds);
}


extern int z_impl_phy_get_link_state(const struct device * dev, struct phy_link_state * state);

__pinned_func
static inline int phy_get_link_state(const struct device * dev, struct phy_link_state * state)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; struct phy_link_state * val; } parm1 = { .val = state };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_PHY_GET_LINK_STATE);
	}
#endif
	compiler_barrier();
	return z_impl_phy_get_link_state(dev, state);
}


extern int z_impl_phy_link_callback_set(const struct device * dev, phy_callback_t callback, void * user_data);

__pinned_func
static inline int phy_link_callback_set(const struct device * dev, phy_callback_t callback, void * user_data)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; phy_callback_t val; } parm1 = { .val = callback };
		union { uintptr_t x; void * val; } parm2 = { .val = user_data };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_PHY_LINK_CALLBACK_SET);
	}
#endif
	compiler_barrier();
	return z_impl_phy_link_callback_set(dev, callback, user_data);
}


extern int z_impl_phy_read(const struct device * dev, uint16_t reg_addr, uint32_t * value);

__pinned_func
static inline int phy_read(const struct device * dev, uint16_t reg_addr, uint32_t * value)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint16_t val; } parm1 = { .val = reg_addr };
		union { uintptr_t x; uint32_t * val; } parm2 = { .val = value };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_PHY_READ);
	}
#endif
	compiler_barrier();
	return z_impl_phy_read(dev, reg_addr, value);
}


extern int z_impl_phy_write(const struct device * dev, uint16_t reg_addr, uint32_t value);

__pinned_func
static inline int phy_write(const struct device * dev, uint16_t reg_addr, uint32_t value)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint16_t val; } parm1 = { .val = reg_addr };
		union { uintptr_t x; uint32_t val; } parm2 = { .val = value };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_PHY_WRITE);
	}
#endif
	compiler_barrier();
	return z_impl_phy_write(dev, reg_addr, value);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
