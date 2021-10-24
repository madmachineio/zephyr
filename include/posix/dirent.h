/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_DIRENT_H_
#define ZEPHYR_INCLUDE_POSIX_DIRENT_H_

#include <limits.h>
#include "posix_types.h"

#ifdef CONFIG_POSIX_FS
#include <fs/fs.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void DIR;

enum {
	DT_UNKNOWN      = 0,
# define DT_UNKNOWN        DT_UNKNOWN
	DT_FIFO         = 1,
# define DT_FIFO        DT_FIFO
	DT_CHR          = 2,
# define DT_CHR                DT_CHR
	DT_DIR          = 4,
# define DT_DIR                DT_DIR
	DT_BLK          = 6,
# define DT_BLK                DT_BLK
	DT_REG          = 8,
# define DT_REG                DT_REG
	DT_LNK          = 10,
# define DT_LNK                DT_LNK
	DT_SOCK         = 12,
# define DT_SOCK        DT_SOCK
	DT_WHT          = 14
# define DT_WHT                DT_WHT
};


struct dirent {
	unsigned int d_ino;
	unsigned char d_type;
	char d_name[PATH_MAX + 1];
};

/* Directory related operations */
extern DIR *opendir(const char *dirname);
extern int closedir(DIR *dirp);
extern struct dirent *readdir(DIR *dirp);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_POSIX_FS */

#endif	/* ZEPHYR_INCLUDE_POSIX_DIRENT_H_ */
