
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_MEM_MANAGE_H
#define Z_INCLUDE_SYSCALLS_MEM_MANAGE_H


#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void z_impl_k_mem_paging_stats_get(struct k_mem_paging_stats_t * stats);

__pinned_func
static inline void k_mem_paging_stats_get(struct k_mem_paging_stats_t * stats)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; struct k_mem_paging_stats_t * val; } parm0 = { .val = stats };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_MEM_PAGING_STATS_GET);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_mem_paging_stats_get(stats);
}


extern void z_impl_k_mem_paging_thread_stats_get(struct k_thread * thread, struct k_mem_paging_stats_t * stats);

__pinned_func
static inline void k_mem_paging_thread_stats_get(struct k_thread * thread, struct k_mem_paging_stats_t * stats)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; struct k_thread * val; } parm0 = { .val = thread };
		union { uintptr_t x; struct k_mem_paging_stats_t * val; } parm1 = { .val = stats };
		(void) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_K_MEM_PAGING_THREAD_STATS_GET);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_mem_paging_thread_stats_get(thread, stats);
}


extern void z_impl_k_mem_paging_histogram_eviction_get(struct k_mem_paging_histogram_t * hist);

__pinned_func
static inline void k_mem_paging_histogram_eviction_get(struct k_mem_paging_histogram_t * hist)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; struct k_mem_paging_histogram_t * val; } parm0 = { .val = hist };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_MEM_PAGING_HISTOGRAM_EVICTION_GET);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_mem_paging_histogram_eviction_get(hist);
}


extern void z_impl_k_mem_paging_histogram_backing_store_page_in_get(struct k_mem_paging_histogram_t * hist);

__pinned_func
static inline void k_mem_paging_histogram_backing_store_page_in_get(struct k_mem_paging_histogram_t * hist)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; struct k_mem_paging_histogram_t * val; } parm0 = { .val = hist };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_IN_GET);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_mem_paging_histogram_backing_store_page_in_get(hist);
}


extern void z_impl_k_mem_paging_histogram_backing_store_page_out_get(struct k_mem_paging_histogram_t * hist);

__pinned_func
static inline void k_mem_paging_histogram_backing_store_page_out_get(struct k_mem_paging_histogram_t * hist)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; struct k_mem_paging_histogram_t * val; } parm0 = { .val = hist };
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_OUT_GET);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_mem_paging_histogram_backing_store_page_out_get(hist);
}


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
