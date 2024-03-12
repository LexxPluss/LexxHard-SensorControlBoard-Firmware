
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_UART_H
#define Z_INCLUDE_SYSCALLS_UART_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_uart_tx(const struct device * dev, const uint8_t * buf, size_t len, int32_t timeout);

__pinned_func
static inline int uart_tx(const struct device * dev, const uint8_t * buf, size_t len, int32_t timeout)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; const uint8_t * val; } parm1 = { .val = buf };
		union { uintptr_t x; size_t val; } parm2 = { .val = len };
		union { uintptr_t x; int32_t val; } parm3 = { .val = timeout };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_UART_TX);
	}
#endif
	compiler_barrier();
	return z_impl_uart_tx(dev, buf, len, timeout);
}


extern int z_impl_uart_tx_abort(const struct device * dev);

__pinned_func
static inline int uart_tx_abort(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_TX_ABORT);
	}
#endif
	compiler_barrier();
	return z_impl_uart_tx_abort(dev);
}


extern int z_impl_uart_rx_enable(const struct device * dev, uint8_t * buf, size_t len, int32_t timeout);

__pinned_func
static inline int uart_rx_enable(const struct device * dev, uint8_t * buf, size_t len, int32_t timeout)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint8_t * val; } parm1 = { .val = buf };
		union { uintptr_t x; size_t val; } parm2 = { .val = len };
		union { uintptr_t x; int32_t val; } parm3 = { .val = timeout };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_UART_RX_ENABLE);
	}
#endif
	compiler_barrier();
	return z_impl_uart_rx_enable(dev, buf, len, timeout);
}


extern int z_impl_uart_rx_disable(const struct device * dev);

__pinned_func
static inline int uart_rx_disable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_RX_DISABLE);
	}
#endif
	compiler_barrier();
	return z_impl_uart_rx_disable(dev);
}


extern int z_impl_uart_err_check(const struct device * dev);

__pinned_func
static inline int uart_err_check(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_ERR_CHECK);
	}
#endif
	compiler_barrier();
	return z_impl_uart_err_check(dev);
}


extern int z_impl_uart_poll_in(const struct device * dev, unsigned char * p_char);

__pinned_func
static inline int uart_poll_in(const struct device * dev, unsigned char * p_char)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; unsigned char * val; } parm1 = { .val = p_char };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_UART_POLL_IN);
	}
#endif
	compiler_barrier();
	return z_impl_uart_poll_in(dev, p_char);
}


extern void z_impl_uart_poll_out(const struct device * dev, unsigned char out_char);

__pinned_func
static inline void uart_poll_out(const struct device * dev, unsigned char out_char)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; unsigned char val; } parm1 = { .val = out_char };
		(void) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_UART_POLL_OUT);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_poll_out(dev, out_char);
}


extern int z_impl_uart_configure(const struct device * dev, const struct uart_config * cfg);

__pinned_func
static inline int uart_configure(const struct device * dev, const struct uart_config * cfg)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; const struct uart_config * val; } parm1 = { .val = cfg };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_UART_CONFIGURE);
	}
#endif
	compiler_barrier();
	return z_impl_uart_configure(dev, cfg);
}


extern int z_impl_uart_config_get(const struct device * dev, struct uart_config * cfg);

__pinned_func
static inline int uart_config_get(const struct device * dev, struct uart_config * cfg)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; struct uart_config * val; } parm1 = { .val = cfg };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_UART_CONFIG_GET);
	}
#endif
	compiler_barrier();
	return z_impl_uart_config_get(dev, cfg);
}


extern void z_impl_uart_irq_tx_enable(const struct device * dev);

__pinned_func
static inline void uart_irq_tx_enable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_TX_ENABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_tx_enable(dev);
}


extern void z_impl_uart_irq_tx_disable(const struct device * dev);

__pinned_func
static inline void uart_irq_tx_disable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_TX_DISABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_tx_disable(dev);
}


extern void z_impl_uart_irq_rx_enable(const struct device * dev);

__pinned_func
static inline void uart_irq_rx_enable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_RX_ENABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_rx_enable(dev);
}


extern void z_impl_uart_irq_rx_disable(const struct device * dev);

__pinned_func
static inline void uart_irq_rx_disable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_RX_DISABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_rx_disable(dev);
}


extern void z_impl_uart_irq_err_enable(const struct device * dev);

__pinned_func
static inline void uart_irq_err_enable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_ERR_ENABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_err_enable(dev);
}


extern void z_impl_uart_irq_err_disable(const struct device * dev);

__pinned_func
static inline void uart_irq_err_disable(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_ERR_DISABLE);
		return;
	}
#endif
	compiler_barrier();
	z_impl_uart_irq_err_disable(dev);
}


extern int z_impl_uart_irq_is_pending(const struct device * dev);

__pinned_func
static inline int uart_irq_is_pending(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_IS_PENDING);
	}
#endif
	compiler_barrier();
	return z_impl_uart_irq_is_pending(dev);
}


extern int z_impl_uart_irq_update(const struct device * dev);

__pinned_func
static inline int uart_irq_update(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_UART_IRQ_UPDATE);
	}
#endif
	compiler_barrier();
	return z_impl_uart_irq_update(dev);
}


extern int z_impl_uart_line_ctrl_set(const struct device * dev, uint32_t ctrl, uint32_t val);

__pinned_func
static inline int uart_line_ctrl_set(const struct device * dev, uint32_t ctrl, uint32_t val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = ctrl };
		union { uintptr_t x; uint32_t val; } parm2 = { .val = val };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_UART_LINE_CTRL_SET);
	}
#endif
	compiler_barrier();
	return z_impl_uart_line_ctrl_set(dev, ctrl, val);
}


extern int z_impl_uart_line_ctrl_get(const struct device * dev, uint32_t ctrl, uint32_t * val);

__pinned_func
static inline int uart_line_ctrl_get(const struct device * dev, uint32_t ctrl, uint32_t * val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = ctrl };
		union { uintptr_t x; uint32_t * val; } parm2 = { .val = val };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_UART_LINE_CTRL_GET);
	}
#endif
	compiler_barrier();
	return z_impl_uart_line_ctrl_get(dev, ctrl, val);
}


extern int z_impl_uart_drv_cmd(const struct device * dev, uint32_t cmd, uint32_t p);

__pinned_func
static inline int uart_drv_cmd(const struct device * dev, uint32_t cmd, uint32_t p)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; uint32_t val; } parm1 = { .val = cmd };
		union { uintptr_t x; uint32_t val; } parm2 = { .val = p };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_UART_DRV_CMD);
	}
#endif
	compiler_barrier();
	return z_impl_uart_drv_cmd(dev, cmd, p);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */