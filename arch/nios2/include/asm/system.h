#ifndef _NIOS2_SYSTEM_H
#define _NIOS2_SYSTEM_H

#include <linux/linkage.h>
#include <linux/compiler.h>
#include <asm/segment.h>
#include <asm/entry.h>
#include <asm/nios2.h>

/*
 * switch_to(n) should switch tasks to task ptr, first checking that
 * ptr isn't the current task, in which case it does nothing.  This
 * also clears the TS-flag if the task we switched to has used the
 * math co-processor latest.
 */

/*
 */
asmlinkage void resume(void);
#define switch_to(prev,next,last)				\
{								\
  void *_last;							\
  __asm__ __volatile__(						\
  	"mov	r4, %1\n"					\
	"mov	r5, %2\n"					\
	"call	resume\n"					\
	"mov	%0,r4\n"					\
       : "=r" (_last)						\
       : "r" (prev), "r" (next)					\
       : "r4","r5","r7","r8","ra");	\
  (last) = _last;						\
}

#define local_irq_enable() __asm__ __volatile__ (  \
	"rdctl	r8, status\n"			   \
	"ori	r8, r8, 1\n"			   \
	"wrctl	status, r8\n"			   \
	: : : "r8")	  

#define local_irq_disable() __asm__ __volatile__ ( \
	"rdctl	r8, status\n"			   \
	"andi	r8, r8, 0xfffe\n"		   \
	"wrctl	status, r8\n"			   \
	: : : "r8")

#define local_save_flags(x) __asm__ __volatile__ (	\
	"rdctl	r8, status\n"				\
	"mov	%0, r8\n"				\
	:"=r" (x) : : "r8", "memory")

#define local_irq_restore(x) __asm__ __volatile__ (	\
	"mov	r8, %0\n"				\
	"wrctl	status, r8\n"				\
	: :"r" (x) : "r8", "memory")

/* For spinlocks etc */
#define local_irq_save(x) do { local_save_flags(x); local_irq_disable(); } while (0)

#define	irqs_disabled()					\
({							\
	unsigned long flags;				\
	local_save_flags(flags);			\
	((flags & NIOS2_STATUS_PIE_MSK) == 0x0);	\
})

#define iret() __asm__ __volatile__ ("eret": : :"memory", "ea")

/*
 * Force strict CPU ordering.
 * Not really required on m68k...
 */
#define nop()  asm volatile ("nop"::)
#define mb()   asm volatile (""   : : :"memory")
#define rmb()  asm volatile (""   : : :"memory")
#define wmb()  asm volatile (""   : : :"memory")
#define set_rmb(var, value)    do { xchg(&var, value); } while (0)
#define set_mb(var, value)     set_rmb(var, value)
#define set_wmb(var, value)    do { var = value; wmb(); } while (0)

#ifdef CONFIG_SMP
#define smp_mb()	mb()
#define smp_rmb()	rmb()
#define smp_wmb()	wmb()
#define smp_read_barrier_depends()	read_barrier_depends()
#else
#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#define smp_read_barrier_depends()	do { } while(0)
#endif

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))
#define tas(ptr) (xchg((ptr),1))

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
  unsigned long tmp, flags;

  local_irq_save(flags);

  switch (size) {
  case 1:
    __asm__ __volatile__( \
      "ldb	%0, %2\n" \
      "stb	%1, %2\n" \
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  case 2:
    __asm__ __volatile__( \
      "ldh	%0, %2\n" \
      "sth	%1, %2\n" \
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  case 4:
    __asm__ __volatile__( \
      "ldw	%0, %2\n" \
      "stw	%1, %2\n" \
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  }
  local_irq_restore(flags);
  return tmp;
}

#include <asm-generic/cmpxchg-local.h>

/*
 * cmpxchg_local and cmpxchg64_local are atomic wrt current CPU. Always make
 * them available.
 */
#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), (unsigned long)(o),\
			(unsigned long)(n), sizeof(*(ptr))))
#define cmpxchg64_local(ptr, o, n) __cmpxchg64_local_generic((ptr), (o), (n))

#ifndef CONFIG_SMP
#include <asm-generic/cmpxchg.h>
#endif

#endif /* _NIOS2_SYSTEM_H */
