#include <dfs_romfs.h>

const struct romfs_dirent _root_dirent[] = {
};

const struct romfs_dirent romfs_root = {ROMFS_DIRENT_DIR, "/", (rt_uint8_t*) _root_dirent, sizeof(_root_dirent)/sizeof(_root_dirent[0])};

