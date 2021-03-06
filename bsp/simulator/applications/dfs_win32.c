
/*
 * File      : rtthread.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-11-27     prife        the first version
 */
#include <rtthread.h>

#include <dfs_fs.h>
#include <dfs_def.h>
#include <rtdevice.h>
//#include "dfs_win32.h"

#include <io.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <WinError.h>
#include  <windows.h>

/*
 * RT-Thread DFS Interface for win-directory as an disk device
 */
#define FILE_PATH_MAX           256  /* the longest file path */

#define WIN32_DIRDISK_ROOT  "." //"F:\\Project\\svn\\rtt\\trunk\\bsp\\simulator_test"

static int win32_result_to_dfs(int res)
{
    rt_kprintf("win error: %x", res);
    return -1;
}

static int dfs_win32_mount(
    struct dfs_filesystem *fs,
    unsigned long rwflag,
    const void *data)
{
    return 0;
}

static int dfs_win32_unmount(struct dfs_filesystem *fs)
{
    return 0;
}

static int dfs_win32_mkfs(const char *device_name)
{
    return -DFS_STATUS_ENOSYS;
}

static int dfs_win32_statfs(struct dfs_filesystem *fs,
                            struct statfs *buf)
{
    return -DFS_STATUS_ENOSYS;
}

static char *winpath_dirdup(char *des, const char *src)
{
    char *path;
    int i = 0;

    path = rt_malloc(FILE_PATH_MAX);
    if (path == RT_NULL)
        return RT_NULL;

    strcpy(path, des);
    strcat(path, src);

    while (1)
    {
        if (path[i] == 0)
            break;

        if (path[i] == '/')
            path[i] = '\\';

        i++;
    }

    return path;
}

static int dfs_win32_open(struct dfs_fd *file)
{
    int fd;
    int oflag, mode;
    char *file_path;
    int res;

    oflag = file->flags;
    if (oflag & DFS_O_DIRECTORY)   /* operations about dir */
    {
        struct _finddata_t  dir;
        int handle;
        int len;

        file_path = winpath_dirdup(WIN32_DIRDISK_ROOT, file->path);

        if (oflag & DFS_O_CREAT)   /* create a dir*/
        {
            res = CreateDirectory(file_path, NULL);
            if (res == ERROR_ALREADY_EXISTS)
            {
                rt_kprintf("already exists!\n");
                return -DFS_STATUS_EEXIST;
            }
            else if (res == ERROR_PATH_NOT_FOUND)
            {
                rt_kprintf("One or more intermediate directories do not exist!\n");
                return -DFS_STATUS_ENOENT;
            }
        }

        len = strlen(file_path);
        if (file_path[len - 1] != '\\')
        {
            file_path[len] = '\\';
            file_path[len + 1] = 0;
        }

        strcat(file_path, "*.*");
        /* _findfirst ��õ� . */
        if ((handle = _findfirst(file_path, &dir)) == -1)
        {
            rt_free(file_path);
            goto __err;
        }

        /* save this pointer,will used by  dfs_win32_getdents*/
        file->data = handle;
        rt_free(file_path);
        return DFS_STATUS_OK;
    }
    /* regular file operations */
    mode = O_BINARY;
    if (oflag & DFS_O_RDONLY) mode |= O_RDONLY;
    if (oflag & DFS_O_WRONLY) mode |= O_WRONLY;
    if (oflag & DFS_O_RDWR)   mode |= O_RDWR;
    /* Opens the file, if it is existing. If not, a new file is created. */
    if (oflag & DFS_O_CREAT) mode |= O_CREAT;
    /* Creates a new file. If the file is existing, it is truncated and overwritten. */
    if (oflag & DFS_O_TRUNC) mode |= O_TRUNC;
    /* Creates a new file. The function fails if the file is already existing. */
    if (oflag & DFS_O_EXCL) mode |= O_EXCL;

    file_path = winpath_dirdup(WIN32_DIRDISK_ROOT, file->path);
    fd = _open(file_path, mode);
    rt_free(file_path);

    if (fd < 0)
        goto __err;

    /* save this pointer, it will be used when calling read()��write(),
     * flush(), seek(), and will be free when calling close()*/
    file->data = (void *)fd;
    file->pos  = 0;
    file->size = _lseek(fd, 0, SEEK_END);

    if (oflag & DFS_O_APPEND)
    {
        file->pos = file->size;
    }
    else
        _lseek(fd, 0, SEEK_SET);

    return 0;

__err:
    return win32_result_to_dfs(GetLastError());
}

static int dfs_win32_close(struct dfs_fd *file)
{
    int oflag;

    oflag = file->flags;
    if (oflag & DFS_O_DIRECTORY)
    {
        /* operations about dir */
        if (_findclose(file->data) < 0)
            goto __err;

        return 0;
    }

    /* regular file operations */
    if (_close((int)(file->data)) < 0)
        goto __err;

    return 0;

__err:
    return win32_result_to_dfs(GetLastError());
}

static int dfs_win32_ioctl(struct dfs_fd *file, int cmd, void *args)
{
    return -DFS_STATUS_ENOSYS;
}

static int dfs_win32_read(struct dfs_fd *file, void *buf, rt_size_t len)
{
    int fd;
    int char_read;

    fd = (int)(file->data);
    char_read = _read(fd, buf, len);
    if (char_read < 0)
        return win32_result_to_dfs(GetLastError());

    /* update position */
    file->pos = _lseek(fd, 0, SEEK_CUR);
    return char_read;
}

static int dfs_win32_write(struct dfs_fd *file,
                           const void *buf,
                           rt_size_t len)
{
    int fd;
    int char_write;

    fd = (int)(file->data);

    char_write = _write(fd, buf, len);
    if (char_write < 0)
        return win32_result_to_dfs(GetLastError());

    /* update position */
    file->pos = _lseek(fd, 0, SEEK_CUR);
    return char_write;
}

static int dfs_win32_flush(struct dfs_fd *file)
{
    return 0;
}

static int dfs_win32_seek(struct dfs_fd *file,
                          rt_off_t offset)
{
    int result;

    /* set offset as current offset */
    if (file->type == FT_DIRECTORY)
    {
        return -DFS_STATUS_ENOSYS;
    }
    else if (file->type == FT_REGULAR)
    {
        result = _lseek((int)(file->data), offset, SEEK_SET);
        if (result >= 0)
            return offset;
    }

    return win32_result_to_dfs(GetLastError());
}

/* return the size of struct dirent*/
static int dfs_win32_getdents(
    struct dfs_fd *file,
    struct dirent *dirp,
    rt_uint32_t count)
{
    rt_uint32_t index;
    struct dirent *d;
    struct _finddata_t  fileinfo;
    int handle;
    int result;

    handle = (int)(file->data);
    RT_ASSERT(handle != RT_NULL);

    /* round count, count is always 1 */
    count = (count / sizeof(struct dirent)) * sizeof(struct dirent);
    if (count == 0) return -DFS_STATUS_EINVAL;

    index = 0;
    /* usually, the while loop should only be looped only once! */
    while (1)
    {
        d = dirp + index;
        if (_findnext(handle, &fileinfo) != 0) //-1 failed
            goto __err;

        if (fileinfo.attrib & _A_SUBDIR)
            d->d_type = DFS_DT_DIR;  /* directory */
        else
            d->d_type = DFS_DT_REG;

        /* write the rest args of struct dirent* dirp  */
        d->d_namlen = strlen(fileinfo.name);
        d->d_reclen = (rt_uint16_t)sizeof(struct dirent);
        strcpy(d->d_name, fileinfo.name);

        index ++;
        if (index * sizeof(struct dirent) >= count)
            break;
    }
    if (index == 0)
        return 0;

    file->pos += index * sizeof(struct dirent);

    return index * sizeof(struct dirent);

__err:
    if ((result = GetLastError()) == ERROR_NO_MORE_FILES)
        return 0;
    else
        return win32_result_to_dfs(result);
}

static int dfs_win32_unlink(struct dfs_filesystem *fs, const char *path)
{
    int result;
    char *fp;
    fp = winpath_dirdup(WIN32_DIRDISK_ROOT, path);
    if (fp == RT_NULL)
    {
        rt_kprintf("out of memory.\n");
        return -DFS_STATUS_ENOMEM;
    }

    result = GetFileAttributes(fp);
    if (result == INVALID_FILE_ATTRIBUTES)
        goto __err;

    if (result & FILE_ATTRIBUTE_DIRECTORY)//winnt.h
    {
        if (RemoveDirectory(fp) == RT_FALSE)
            goto __err;
    }
    else //(result & FILE_ATTRIBUTE_NORMAL)
    {
        if (_unlink(fp) < 0)
            goto __err;
    }

    rt_free(fp);
    return 0;
__err:
    rt_free(fp);
    return win32_result_to_dfs(GetLastError());
}

static int dfs_win32_rename(
    struct dfs_filesystem *fs,
    const char *oldpath,
    const char *newpath)
{
    int result;
    char *op, *np;
    op = winpath_dirdup(WIN32_DIRDISK_ROOT, oldpath);
    np = winpath_dirdup(WIN32_DIRDISK_ROOT, newpath);
    if (op == RT_NULL || np == RT_NULL)
    {
        rt_kprintf("out of memory.\n");
        return -DFS_STATUS_ENOMEM;
    }

    /* If the function fails, the return value is zero. */
    result = MoveFile(op, np);

    rt_free(op);
    rt_free(np);

    if (result == 0)
        return win32_result_to_dfs(GetLastError());

    return 0;
}

static int dfs_win32_stat(struct dfs_filesystem *fs, const char *path, struct stat *st)
{
    WIN32_FIND_DATA fileInfo;
    HANDLE hFind;
    char *fp;
    fp = winpath_dirdup(WIN32_DIRDISK_ROOT, path);
    if (fp == RT_NULL)
    {
        rt_kprintf("out of memory.\n");
        return -DFS_STATUS_ENOMEM;
    }

    hFind = FindFirstFile(fp, &fileInfo);
    rt_free(fp);

    if (hFind == INVALID_HANDLE_VALUE)
        goto __err;

    st->st_mode = DFS_S_IFREG | DFS_S_IRUSR | DFS_S_IRGRP | DFS_S_IROTH |
                  DFS_S_IWUSR | DFS_S_IWGRP | DFS_S_IWOTH;

    /* convert file info to dfs stat structure */
    if (fileInfo.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
    {
        st->st_mode &= ~DFS_S_IFREG;
        st->st_mode |= DFS_S_IFDIR | DFS_S_IXUSR | DFS_S_IXGRP | DFS_S_IXOTH;
    }

    st->st_dev  = 0;
    st->st_size = fileInfo.nFileSizeLow;
    st->st_mtime = 0;
    st->st_blksize = 0;

    FindClose(hFind);

    return 0;

__err:
    return win32_result_to_dfs(GetLastError());
}

static const struct dfs_filesystem_operation dfs_win32_ops =
{
    "wdir", /* file system type: dir */
    DFS_FS_FLAG_DEFAULT,
    dfs_win32_mount,
    dfs_win32_unmount,
    dfs_win32_mkfs,
    dfs_win32_statfs,

    dfs_win32_open,
    dfs_win32_close,
    dfs_win32_ioctl,
    dfs_win32_read,
    dfs_win32_write,
    dfs_win32_flush,
    dfs_win32_seek,
    dfs_win32_getdents,
    dfs_win32_unlink,
    dfs_win32_stat,
    dfs_win32_rename,
};

int dfs_win32_init(void)
{
    /* register uffs file system */
    dfs_register(&dfs_win32_ops);

    return 0;
}

static rt_err_t nop_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t nop_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t nop_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t nop_read(rt_device_t dev,
                          rt_off_t    pos,
                          void       *buffer,
                          rt_size_t   size)
{
    return size;
}

static rt_size_t nop_write(rt_device_t dev,
                           rt_off_t    pos,
                           const void *buffer,
                           rt_size_t   size)
{
    return size;
}

static rt_err_t nop_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    return RT_EOK;
}

static struct rt_device win_sharedir_dev;
rt_err_t rt_win_sharedir_init(const char *name)
{
    rt_device_t dev;

    dev = &win_sharedir_dev;
    RT_ASSERT(dev != RT_NULL);

    /* set device class and generic device interface */
    dev->type        = RT_Device_Class_Block;
    dev->init        = nop_init;
    dev->open        = nop_open;
    dev->read        = nop_read;
    dev->write       = nop_write;
    dev->close       = nop_close;
    dev->control     = nop_control;

    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;

    /* register to RT-Thread device system */
    return rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}
