
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_LED_H
#define Z_INCLUDE_SYSCALLS_LED_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_led_blink(const struct device * dev, uint32_t led, uint32_t delay_on, uint32_t delay_off);

__pinned_func
static inline int led_blink(const struct device * dev, uint32_t led, uint32_t delay_on, uint32_t delay_off)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		union { uintptr_t x; uint32_t val; } parm2 = { .val = delay_on };
		union { uintptr_t x; uint32_t val; } parm3 = { .val = delay_off };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_LED_BLINK);
	}
#endif
	compiler_barrier();
	return z_impl_led_blink(dev, led, delay_on, delay_off);
}


extern int z_impl_led_get_info(const struct device * dev, uint32_t led, const struct led_info ** info);

__pinned_func
static inline int led_get_info(const struct device * dev, uint32_t led, const struct led_info ** info)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		union { uintptr_t x; const struct led_info ** val; } parm2 = { .val = info };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_LED_GET_INFO);
	}
#endif
	compiler_barrier();
	return z_impl_led_get_info(dev, led, info);
}


extern int z_impl_led_set_brightness(const struct device * dev, uint32_t led, uint8_t value);

__pinned_func
static inline int led_set_brightness(const struct device * dev, uint32_t led, uint8_t value)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		union { uintptr_t x; uint8_t val; } parm2 = { .val = value };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_LED_SET_BRIGHTNESS);
	}
#endif
	compiler_barrier();
	return z_impl_led_set_brightness(dev, led, value);
}


extern int z_impl_led_write_channels(const struct device * dev, uint32_t start_channel, uint32_t num_channels, const uint8_t * buf);

__pinned_func
static inline int led_write_channels(const struct device * dev, uint32_t start_channel, uint32_t num_channels, const uint8_t * buf)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = start_channel };
		union { uintptr_t x; uint32_t val; } parm2 = { .val = num_channels };
		union { uintptr_t x; const uint8_t * val; } parm3 = { .val = buf };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_LED_WRITE_CHANNELS);
	}
#endif
	compiler_barrier();
	return z_impl_led_write_channels(dev, start_channel, num_channels, buf);
}


extern int z_impl_led_set_channel(const struct device * dev, uint32_t channel, uint8_t value);

__pinned_func
static inline int led_set_channel(const struct device * dev, uint32_t channel, uint8_t value)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = channel };
		union { uintptr_t x; uint8_t val; } parm2 = { .val = value };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_LED_SET_CHANNEL);
	}
#endif
	compiler_barrier();
	return z_impl_led_set_channel(dev, channel, value);
}


extern int z_impl_led_set_color(const struct device * dev, uint32_t led, uint8_t num_colors, const uint8_t * color);

__pinned_func
static inline int led_set_color(const struct device * dev, uint32_t led, uint8_t num_colors, const uint8_t * color)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		union { uintptr_t x; uint8_t val; } parm2 = { .val = num_colors };
		union { uintptr_t x; const uint8_t * val; } parm3 = { .val = color };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_LED_SET_COLOR);
	}
#endif
	compiler_barrier();
	return z_impl_led_set_color(dev, led, num_colors, color);
}


extern int z_impl_led_on(const struct device * dev, uint32_t led);

__pinned_func
static inline int led_on(const struct device * dev, uint32_t led)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_LED_ON);
	}
#endif
	compiler_barrier();
	return z_impl_led_on(dev, led);
}


extern int z_impl_led_off(const struct device * dev, uint32_t led);

__pinned_func
static inline int led_off(const struct device * dev, uint32_t led)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = led };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_LED_OFF);
	}
#endif
	compiler_barrier();
	return z_impl_led_off(dev, led);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
