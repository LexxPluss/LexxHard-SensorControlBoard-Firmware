/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/libc-hooks.h>

extern size_t z_vrfy_zephyr_fwrite(const void *_MLIBC_RESTRICT ptr, size_t size, size_t nitems, FILE *_MLIBC_RESTRICT stream);
uintptr_t z_mrsh_zephyr_fwrite(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; const void *_MLIBC_RESTRICT val; } parm0;
	parm0.x = arg0;
	union { uintptr_t x; size_t val; } parm1;
	parm1.x = arg1;
	union { uintptr_t x; size_t val; } parm2;
	parm2.x = arg2;
	union { uintptr_t x; FILE *_MLIBC_RESTRICT val; } parm3;
	parm3.x = arg3;
	size_t ret = z_vrfy_zephyr_fwrite(parm0.val, parm1.val, parm2.val, parm3.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}

