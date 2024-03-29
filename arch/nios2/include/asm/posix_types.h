#ifndef _ASM_NIOS2_POSIX_TYPES_H
#define _ASM_NIOS2_POSIX_TYPES_H

typedef unsigned short	__kernel_mode_t;
#define __kernel_mode_t __kernel_mode_t
typedef unsigned short	__kernel_nlink_t;
#define __kernel_nlink_t __kernel_nlink_t
typedef unsigned short	__kernel_ipc_pid_t;
#define __kernel_ipc_pid_t __kernel_ipc_pid_t
typedef unsigned short	__kernel_uid_t;
#define __kernel_uid_t __kernel_uid_t
typedef unsigned short	__kernel_gid_t;
#define __kernel_gid_t __kernel_gid_t

typedef unsigned int	__kernel_uid32_t;
#define __kernel_uid32_t __kernel_uid32_t
typedef unsigned int	__kernel_gid32_t;
#define __kernel_gid32_t __kernel_gid32_t

typedef unsigned short	__kernel_old_uid_t;
#define __kernel_old_uid_t __kernel_old_uid_t
typedef unsigned short	__kernel_old_gid_t;
#define __kernel_old_gid_t __kernel_old_gid_t
typedef unsigned short	__kernel_old_dev_t;
#define __kernel_old_dev_t __kernel_old_dev_t

#include <asm-generic/posix_types.h>

#endif /* _ASM_NIOS2_POSIX_TYPES_H */
