#ifndef __NIOS2_UACCESS_H
#define __NIOS2_UACCESS_H

/*--------------------------------------------------------------------
 *
 * asm-nios2/uaccess.h
 *
 * User space memory access functions
 *
 * Derived from various works, Alpha, ix86, M68K, Sparc, ...et al
 *
 * Copyright (C) 2004   Microtronix Datacom Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Ported from asm-m68knommu/uaccess.h --wentao
 * Jan/20/2004		dgt	    NiosII
 *
 ---------------------------------------------------------------------*/


#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/segment.h>
#include <asm/nios2.h>

#define VERIFY_READ	0
#define VERIFY_WRITE	1

#define access_ok(type,addr,size)	_access_ok((unsigned long)(addr),(size))

static inline int _access_ok(unsigned long addr, unsigned long size)
{
	addr &= ~0x80000000;	/* ignore 'uncached' bit */
	return ((addr + size <= (unsigned long)nasys_program_mem_end) &&
		((addr >= (unsigned long)nasys_program_mem)));
}

/*
 * The exception table consists of pairs of addresses: the first is the
 * address of an instruction that is allowed to fault, and the second is
 * the address at which the program should continue.  No registers are
 * modified, so it is entirely up to the continuation code to figure out
 * what to do.
 *
 * All the routines below use bits of fixup code that are out of line
 * with the main instruction path.  This means when everything is well,
 * we don't even have to jump over them.  Further, they do not intrude
 * on our cache or tlb entries.
 */

#define ARCH_HAS_SEARCH_EXTABLE
//;dgt2;tmp;

struct exception_table_entry
{
	unsigned long insn, fixup;
};

/* Returns 0 if exception not found and fixup otherwise.  */
extern unsigned long search_exception_table(unsigned long);


/*
 * These are the main single-value transfer routines.  They automatically
 * use the right size if we just have the right pointer type.
 */

#define put_user(x, ptr)				\
({							\
    int __pu_err = 0;					\
    typeof(*(ptr)) __pu_val = (x);			\
    switch (sizeof (*(ptr))) {				\
    case 1:						\
    case 2:						\
    case 4:						\
    case 8:						\
	memcpy(ptr, &__pu_val, sizeof (*(ptr))); \
	break;						\
    default:						\
	__pu_err = __put_user_bad();			\
	break;						\
    }							\
    __pu_err;						\
})
#define __put_user(x, ptr) put_user(x, ptr)

extern int __put_user_bad(void);

/*
 * Tell gcc we read from memory instead of writing: this is because
 * we do not write to any memory gcc knows about, so there are no
 * aliasing issues.
 */

#define __ptr(x) ((unsigned long *)(x))

#define get_user(x, ptr)					\
({								\
    int __gu_err = 0;						\
    typeof(*(ptr)) __gu_val = 0;				\
    switch (sizeof(*(ptr))) {					\
    case 1:						\
    case 2:						\
    case 4:						\
    case 8:						\
	memcpy(&__gu_val, ptr, sizeof (*(ptr))); \
	break;							\
    default:							\
	__gu_val = 0;						\
	__gu_err = __get_user_bad();				\
	break;							\
    }								\
    (x) = __gu_val;						\
    __gu_err;							\
})
#define __get_user(x, ptr) get_user(x, ptr)

extern int __get_user_bad(void);

#define copy_from_user(to, from, n)		(memcpy(to, from, n), 0)
#define copy_to_user(to, from, n)		(memcpy(to, from, n), 0)

#define __copy_from_user(to, from, n) copy_from_user(to, from, n)
#define __copy_to_user(to, from, n) copy_to_user(to, from, n)
#define __copy_to_user_inatomic __copy_to_user
#define __copy_from_user_inatomic __copy_from_user

#define copy_to_user_ret(to,from,n,retval) ({ if (copy_to_user(to,from,n)) return retval; })

#define copy_from_user_ret(to,from,n,retval) ({ if (copy_from_user(to,from,n)) return retval; })

/*
 * Copy a null terminated string from userspace.
 */

static inline long
strncpy_from_user(char *dst, const char *src, long count)
{
	char *tmp;
	strncpy(dst, src, count);
	for (tmp = dst; *tmp && count > 0; tmp++, count--)
		;
	return(tmp - dst); /* DAVIDM should we count a NUL ?  check getname */
}

/*
 * Return the size of a string (including the ending 0)
 *
 * Return 0 on exception, a value greater than N if too long
 */
static inline long strnlen_user(const char *src, long n)
{
	return(strlen(src) + 1); /* DAVIDM make safer */
}

#define strlen_user(str) strnlen_user(str, 32767)

/*
 * Zero Userspace
 */

static inline unsigned long __clear_user(void *to, unsigned long n)
{
	memset(to, 0, n);
	return(0);
}

#define clear_user(to, n) __clear_user(to, n)

#endif /* _NIOS2_UACCESS_H */
