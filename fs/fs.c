/*
 * fs.c - posix like file functions
 *
 * Copyright (c) 2007 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <common.h>
#include <fs.h>
#include <driver.h>
#include <errno.h>
#include <malloc.h>
#include <linux/stat.h>
#include <linux/err.h>
#include <fcntl.h>
#include <xfuncs.h>
#include <init.h>
#include <module.h>
#include <libbb.h>
#include <magicvar.h>
#include <environment.h>
#include <libgen.h>

void *read_file(const char *filename, size_t *size)
{
	int fd;
	struct stat s;
	void *buf = NULL;

	if (stat(filename, &s))
		return NULL;

	buf = xzalloc(s.st_size + 1);

	fd = open(filename, O_RDONLY);
	if (fd < 0)
		goto err_out;

	if (read(fd, buf, s.st_size) < s.st_size)
		goto err_out1;

	close(fd);

	if (size)
		*size = s.st_size;

	return buf;

err_out1:
	close(fd);
err_out:
	free(buf);
	return NULL;
}

EXPORT_SYMBOL(read_file);

int write_file(const char *filename, void *buf, size_t size)
{
	int fd, ret;

	fd = open(filename, O_WRONLY | O_TRUNC | O_CREAT);
	if (fd < 0)
		return fd;

	ret = write_full(fd, buf, size);

	close(fd);

	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(write_file);

char *mkmodestr(unsigned long mode, char *str)
{
	static const char *l = "xwr";
	int mask = 1, i;
	char c;

	switch (mode & S_IFMT) {
		case S_IFDIR:    str[0] = 'd'; break;
		case S_IFBLK:    str[0] = 'b'; break;
		case S_IFCHR:    str[0] = 'c'; break;
		case S_IFIFO:    str[0] = 'f'; break;
		case S_IFLNK:    str[0] = 'l'; break;
		case S_IFSOCK:   str[0] = 's'; break;
		case S_IFREG:    str[0] = '-'; break;
		default:         str[0] = '?';
	}

	for(i = 0; i < 9; i++) {
		c = l[i%3];
		str[9-i] = (mode & mask)?c:'-';
		mask = mask<<1;
	}

	if(mode & S_ISUID) str[3] = (mode & S_IXUSR)?'s':'S';
	if(mode & S_ISGID) str[6] = (mode & S_IXGRP)?'s':'S';
	if(mode & S_ISVTX) str[9] = (mode & S_IXOTH)?'t':'T';
	str[10] = '\0';
	return str;
}
EXPORT_SYMBOL(mkmodestr);

static char *cwd;

static FILE *files;

static int init_fs(void)
{
	cwd = xzalloc(PATH_MAX);
	*cwd = '/';

	files = xzalloc(sizeof(FILE) * MAX_FILES);

	return 0;
}

postcore_initcall(init_fs);

char *normalise_link(const char *pathname, const char *symlink)
{
	const char *buf = symlink;
	char *path_free, *path;
	char *absolute_path;
	int point = 0;
	int dir = 1;
	int len;

	if (symlink[0] == '/')
		return strdup(symlink);

	while (*buf == '.' || *buf == '/') {
		if (*buf == '.') {
			point++;
		} else if (*buf == '/') {
			point = 0;
			dir++;
		}
		if (point > 2) {
			buf -= 2;
			break;
		}
		buf++;
	}

	path = path_free = strdup(pathname);
	if (!path)
		return NULL;

	while(dir) {
		path = dirname(path);
		dir--;
	}

	len = strlen(buf) + strlen(path) + 1;
	if (buf[0] != '/')
		len++;

	absolute_path = calloc(sizeof(char), len);

	if (!absolute_path)
		goto out;

	strcat(absolute_path, path);
	if (buf[0] != '/')
		strcat(absolute_path, "/");
	strcat(absolute_path, buf);

out:
	free(path_free);

	return absolute_path;
}

char *normalise_path(const char *pathname)
{
	char *path = xzalloc(strlen(pathname) + strlen(cwd) + 2);
        char *in, *out, *slashes[32];
	int sl = 0;

	debug("in: %s\n", pathname);

	if (*pathname != '/')
		strcpy(path, cwd);
	strcat(path, "/");
	strcat(path, pathname);

	slashes[0] = in = out = path;

        while (*in) {
                if(*in == '/') {
			slashes[sl++] = out;
                        *out++ = *in++;
                        while(*in == '/')
                                in++;
                } else {
			if (*in == '.' && (*(in + 1) == '/' || !*(in + 1))) {
				sl--;
				if (sl < 0)
					sl = 0;
				out = slashes[sl];
				in++;
				continue;
			}
			if (*in == '.' && *(in + 1) == '.') {
				sl -= 2;
				if (sl < 0)
					sl = 0;
				out = slashes[sl];
				in += 2;
				continue;
			}
                        *out++ = *in++;
                }
        }

	*out-- = 0;

        /*
         * Remove trailing slash
         */
        if (*out == '/')
                *out = 0;

	if (!*path) {
		*path = '/';
		*(path + 1) = 0;
	}

	return path;
}
EXPORT_SYMBOL(normalise_path);

LIST_HEAD(fs_device_list);
static struct fs_device_d *fs_dev_root;

static struct fs_device_d *get_fsdevice_by_path(const char *path)
{
	struct fs_device_d *fsdev = NULL;

	for_each_fs_device(fsdev) {
		int len = strlen(fsdev->path);
		if (!strncmp(path, fsdev->path, len) &&
				(path[len] == '/' || path[len] == 0))
			return fsdev;
	}

	return fs_dev_root;
}

char *get_mounted_path(const char *path)
{
	struct fs_device_d *fdev;

	fdev = get_fsdevice_by_path(path);

	return fdev->path;
}

static FILE *get_file(void)
{
	int i;

	for (i = 3; i < MAX_FILES; i++) {
		if (!files[i].in_use) {
			memset(&files[i], 0, sizeof(FILE));
			files[i].in_use = 1;
			files[i].no = i;
			return &files[i];
		}
	}
	return NULL;
}

static void put_file(FILE *f)
{
	files[f->no].in_use = 0;
}

static int check_fd(int fd)
{
	if (fd < 0 || fd >= MAX_FILES || !files[fd].in_use) {
		errno = EBADF;
		return -errno;
	}

	return 0;
}

#ifdef CONFIG_FS_AUTOMOUNT

#define AUTOMOUNT_IS_FILE (1 << 0)

struct automount {
	char *path;
	char *cmd;
	struct list_head list;
	unsigned int flags;
};

static LIST_HEAD(automount_list);

void automount_remove(const char *_path)
{
	char *path = normalise_path(_path);
	struct automount *am;

	list_for_each_entry(am, &automount_list, list) {
		if (!strcmp(path, am->path))
			goto found;
	}

	return;
found:
	list_del(&am->list);
	free(am->path);
	free(am->cmd);
	free(am);
}
EXPORT_SYMBOL(automount_remove);

int automount_add(const char *path, const char *cmd)
{
	struct automount *am = xzalloc(sizeof(*am));
	struct stat s;
	int ret;

	am->path = normalise_path(path);
	am->cmd = xstrdup(cmd);

	automount_remove(am->path);

	ret = stat(path, &s);
	if (!ret) {
		/*
		 * If it exists it must be a directory
		 */
		if (!S_ISDIR(s.st_mode))
			return -ENOTDIR;
	} else {
		am->flags |= AUTOMOUNT_IS_FILE;
	}

	list_add_tail(&am->list, &automount_list);

	return 0;
}
EXPORT_SYMBOL(automount_add);

void automount_print(void)
{
	struct automount *am;

	list_for_each_entry(am, &automount_list, list)
		printf("%-20s %s\n", am->path, am->cmd);
}
EXPORT_SYMBOL(automount_print);

static void automount_mount(const char *path, int instat)
{
	struct automount *am;
	int ret;

	list_for_each_entry(am, &automount_list, list) {
		int len_path = strlen(path);
		int len_am_path = strlen(am->path);

		/*
		 * stat is a bit special. We do not want to trigger
		 * automount when someone calls stat() on the automount
		 * directory itself.
		 */
		if (instat && !(am->flags & AUTOMOUNT_IS_FILE) &&
				len_path == len_am_path) {
			continue;
		}

		if (len_path < len_am_path)
			continue;

		if (strncmp(path, am->path, len_am_path))
			continue;

		if (*(path + len_am_path) != 0 && *(path + len_am_path) != '/')
			continue;

		setenv("automount_path", am->path);
		export("automount_path");
		ret = run_command(am->cmd, 0);
		setenv("automount_path", NULL);

		if (ret)
			printf("running automount command '%s' failed\n",
					am->cmd);
		else
			automount_remove(am->path);

		return;
	}
}

BAREBOX_MAGICVAR(automount_path, "mountpath passed to automount scripts");

#else
static void automount_mount(const char *path, int instat)
{
}
#endif /* CONFIG_FS_AUTOMOUNT */

static struct fs_device_d *get_fs_device_and_root_path(char **path)
{
	struct fs_device_d *fsdev;

	automount_mount(*path, 0);

	fsdev = get_fsdevice_by_path(*path);
	if (!fsdev)
		return NULL;
	if (fsdev != fs_dev_root)
		*path += strlen(fsdev->path);

	return fsdev;
}

static int dir_is_empty(const char *pathname)
{
	DIR *dir;
	struct dirent *d;
	int ret = 1;

	dir = opendir(pathname);
	if (!dir) {
		errno = ENOENT;
		return -ENOENT;
	}

	while ((d = readdir(dir))) {
		if (!strcmp(d->d_name, ".") || !strcmp(d->d_name, ".."))
				continue;
		ret = 0;
		break;
	}

	closedir(dir);
	return ret;
}

#define S_UB_IS_EMPTY		(1 << 31)
#define S_UB_EXISTS		(1 << 30)
#define S_UB_DOES_NOT_EXIST	(1 << 29)

/*
 * Helper function to check the prerequisites of a path given
 * to fs functions. Besides the flags above S_IFREG and S_IFDIR
 * can be passed in.
 */
static int path_check_prereq(const char *path, unsigned int flags)
{
	struct stat s;
	unsigned int m;
	int ret = 0;

	if (lstat(path, &s)) {
		if (flags & S_UB_DOES_NOT_EXIST)
			goto out;
		ret = -ENOENT;
		goto out;
	}

	if (flags & S_UB_DOES_NOT_EXIST) {
		ret = -EEXIST;
		goto out;
	}

	if (flags == S_UB_EXISTS)
		goto out;

	m = s.st_mode;

	if (S_ISDIR(m)) {
		if (flags & S_IFREG) {
			ret = -EISDIR;
			goto out;
		}
		if ((flags & S_UB_IS_EMPTY) && !dir_is_empty(path)) {
			ret = -ENOTEMPTY;
			goto out;
		}
	}
	if ((flags & S_IFDIR) && S_ISREG(m)) {
		ret = -ENOTDIR;
		goto out;
	}

out:
	return ret;
}

static int parent_check_directory(const char *path)
{
	struct stat s;
	int ret;
	char *dir = dirname(xstrdup(path));

	ret = lstat(dir, &s);

	free(dir);

	if (ret)
		return -ENOENT;

	if (!S_ISDIR(s.st_mode))
		return -ENOTDIR;

	return 0;
}

const char *getcwd(void)
{
	return cwd;
}
EXPORT_SYMBOL(getcwd);

int chdir(const char *pathname)
{
	char *p = normalise_path(pathname);
	int ret;


	ret = path_check_prereq(p, S_IFDIR);
	if (ret)
		goto out;

	strcpy(cwd, p);

out:
	free(p);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(chdir);

int unlink(const char *pathname)
{
	struct fs_device_d *fsdev;
	struct fs_driver_d *fsdrv;
	char *p = normalise_path(pathname);
	char *freep = p;
	int ret;

	ret = path_check_prereq(pathname, S_IFREG);
	if (ret) {
		ret = -EINVAL;
		goto out;
	}

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENOENT;
		goto out;
	}
	fsdrv = fsdev->driver;

	if (!fsdrv->unlink) {
		ret = -ENOSYS;
		goto out;
	}

	ret = fsdrv->unlink(&fsdev->dev, p);
	if (ret)
		errno = -ret;
out:
	free(freep);
	if (ret)
		errno = -ret;
	return ret;
}
EXPORT_SYMBOL(unlink);

static char *realfile(const char *pathname, struct stat *s)
{
	char *path = normalise_path(pathname);
	int ret;

	ret = lstat(path, s);
	if (ret)
		goto out;

	if (S_ISLNK(s->st_mode)) {
		char tmp[PATH_MAX];
		char *new_path;

		memset(tmp, 0, PATH_MAX);

		ret = readlink(path, tmp, PATH_MAX - 1);
		if (ret < 0)
			goto out;

		new_path = normalise_link(path, tmp);
		free(path);
		if (!new_path)
			return ERR_PTR(-ENOMEM);
		path = new_path;

		ret = lstat(path, s);
	}

	if (!ret)
		return path;

out:
	free(path);
	return ERR_PTR(ret);
}

int open(const char *pathname, int flags, ...)
{
	struct fs_device_d *fsdev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int exist_err = 0;
	struct stat s;
	char *path;
	char *freep;
	int ret;

	path = realfile(pathname, &s);

	if (IS_ERR(path)) {
		exist_err = PTR_ERR(path);
		path = normalise_path(pathname);
	}

	freep = path;

	if (!exist_err && S_ISDIR(s.st_mode)) {
		ret = -EISDIR;
		goto out1;
	}

	if (exist_err && !(flags & O_CREAT)) {
		ret = exist_err;
		goto out1;
	}

	if (exist_err) {
		ret = parent_check_directory(path);
		if (ret)
			goto out1;
	}

	f = get_file();
	if (!f) {
		ret = -EMFILE;
		goto out1;
	}

	fsdev = get_fs_device_and_root_path(&path);
	if (!fsdev) {
		ret = -ENOENT;
		goto out;
	}

	fsdrv = fsdev->driver;

	f->dev = &fsdev->dev;
	f->flags = flags;

	if ((flags & O_ACCMODE) && !fsdrv->write) {
		ret = -EROFS;
		goto out;
	}

	if (exist_err) {
		if (NULL != fsdrv->create)
			ret = fsdrv->create(&fsdev->dev, path,
					S_IFREG | S_IRWXU | S_IRWXG | S_IRWXO);
		else
			ret = -EROFS;
		if (ret)
			goto out;
	}
	ret = fsdrv->open(&fsdev->dev, f, path);
	if (ret)
		goto out;


	if (flags & O_TRUNC) {
		ret = fsdrv->truncate(&fsdev->dev, f, 0);
		f->size = 0;
		if (ret)
			goto out;
	}

	if (flags & O_APPEND)
		f->pos = f->size;

	free(freep);
	return f->no;

out:
	put_file(f);
out1:
	free(freep);
	if (ret)
		errno = -ret;
	return ret;
}
EXPORT_SYMBOL(open);

int creat(const char *pathname, mode_t mode)
{
	return open(pathname, O_CREAT | O_WRONLY | O_TRUNC);
}
EXPORT_SYMBOL(creat);

int ioctl(int fd, int request, void *buf)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];
	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);

	if (fsdrv->ioctl)
		ret = fsdrv->ioctl(dev, f, request, buf);
	else
		ret = -ENOSYS;
	if (ret)
		errno = -ret;
	return ret;
}

static ssize_t __read(FILE *f, void *buf, size_t count)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	int ret;

	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);

	if (f->size != FILE_SIZE_STREAM && f->pos + count > f->size)
		count = f->size - f->pos;

	if (!count)
		return 0;

	ret = fsdrv->read(dev, f, buf, count);

	if (ret < 0)
		errno = -ret;
	return ret;
}

ssize_t pread(int fd, void *buf, size_t count, loff_t offset)
{
	loff_t pos;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];

	pos = f->pos;
	f->pos = offset;
	ret = __read(f, buf, count);
	f->pos = pos;

	return ret;
}
EXPORT_SYMBOL(pread);

ssize_t read(int fd, void *buf, size_t count)
{
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];

	ret = __read(f, buf, count);

	if (ret > 0)
		f->pos += ret;
	return ret;
}
EXPORT_SYMBOL(read);

static ssize_t __write(FILE *f, const void *buf, size_t count)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	int ret;

	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);
	if (f->size != FILE_SIZE_STREAM && f->pos + count > f->size) {
		ret = fsdrv->truncate(dev, f, f->pos + count);
		if (ret) {
			if (ret != -ENOSPC)
				goto out;
			count = f->size - f->pos;
			if (!count)
				goto out;
		} else {
			f->size = f->pos + count;
		}
	}
	ret = fsdrv->write(dev, f, buf, count);
out:
	if (ret < 0)
		errno = -ret;
	return ret;
}

ssize_t pwrite(int fd, const void *buf, size_t count, loff_t offset)
{
	loff_t pos;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];

	pos = f->pos;
	f->pos = offset;
	ret = __write(f, buf, count);
	f->pos = pos;

	return ret;
}
EXPORT_SYMBOL(pwrite);

ssize_t write(int fd, const void *buf, size_t count)
{
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];

	ret = __write(f, buf, count);

	if (ret > 0)
		f->pos += ret;
	return ret;
}
EXPORT_SYMBOL(write);

int flush(int fd)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];
	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);
	if (fsdrv->flush)
		ret = fsdrv->flush(dev, f);
	else
		ret = 0;

	if (ret)
		errno = -ret;

	return ret;
}

loff_t lseek(int fildes, loff_t offset, int whence)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	loff_t pos;
	int ret;

	if (check_fd(fildes))
		return -1;

	f = &files[fildes];
	dev = f->dev;
	fsdrv = dev_to_fs_driver(dev);
	if (!fsdrv->lseek) {
		ret = -ENOSYS;
		goto out;
	}

	ret = -EINVAL;

	switch (whence) {
	case SEEK_SET:
		if (f->size != FILE_SIZE_STREAM && offset > f->size)
			goto out;
		pos = offset;
		break;
	case SEEK_CUR:
		if (f->size != FILE_SIZE_STREAM && offset + f->pos > f->size)
			goto out;
		pos = f->pos + offset;
		break;
	case SEEK_END:
		if (offset)
			goto out;
		pos = f->size;
		break;
	default:
		goto out;
	}

	return fsdrv->lseek(dev, f, pos);

out:
	if (ret)
		errno = -ret;

	return -1;
}
EXPORT_SYMBOL(lseek);

int erase(int fd, size_t count, unsigned long offset)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;
	f = &files[fd];
	if (offset >= f->size)
		return 0;
	if (count > f->size - offset)
		count = f->size - offset;

	dev = f->dev;
	fsdrv = dev_to_fs_driver(dev);
	if (fsdrv->erase)
		ret = fsdrv->erase(dev, f, count, offset);
	else
		ret = -ENOSYS;

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(erase);

int protect(int fd, size_t count, unsigned long offset, int prot)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;
	f = &files[fd];
	if (offset >= f->size)
		return 0;
	if (count > f->size - offset)
		count = f->size - offset;

	dev = f->dev;
	fsdrv = dev_to_fs_driver(dev);
	if (fsdrv->protect)
		ret = fsdrv->protect(dev, f, count, offset, prot);
	else
		ret = -ENOSYS;

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(protect);

int protect_file(const char *file, int prot)
{
	int fd, ret;

	fd = open(file, O_WRONLY);
	if (fd < 0)
		return fd;

	ret = protect(fd, ~0, 0, prot);

	close(fd);

	return ret;
}

void *memmap(int fd, int flags)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	void *retp = (void *)-1;
	int ret;

	if (check_fd(fd))
		return retp;

	f = &files[fd];
	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);

	if (fsdrv->memmap)
		ret = fsdrv->memmap(dev, f, &retp, flags);
	else
		ret = -EINVAL;

	if (ret)
		errno = -ret;

	return retp;
}
EXPORT_SYMBOL(memmap);

int close(int fd)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	FILE *f;
	int ret;

	if (check_fd(fd))
		return -errno;

	f = &files[fd];
	dev = f->dev;

	fsdrv = dev_to_fs_driver(dev);
	ret = fsdrv->close(dev, f);

	put_file(f);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(close);

int readlink(const char *pathname, char *buf, size_t bufsiz)
{
	struct fs_driver_d *fsdrv;
	struct fs_device_d *fsdev;
	char *p = normalise_path(pathname);
	char *freep = p;
	int ret;

	ret = path_check_prereq(pathname, S_IFLNK);
	if (ret)
		goto out;

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENODEV;
		goto out;
	}
	fsdrv = fsdev->driver;

	if (fsdrv->readlink)
		ret = fsdrv->readlink(&fsdev->dev, p, buf, bufsiz);
	else
		ret = -ENOSYS;

	if (ret)
		goto out;

out:
	free(freep);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(readlink);

int symlink(const char *pathname, const char *newpath)
{
	struct fs_driver_d *fsdrv;
	struct fs_device_d *fsdev;
	char *p;
	char *freep = normalise_path(pathname);
	int ret;
	struct stat s;

	if (!freep)
		return -ENOMEM;

	if (!stat(freep, &s) && S_ISDIR(s.st_mode)) {
		ret = -ENOSYS;
		goto out;
	}

	free(freep);
	freep = p = normalise_path(newpath);

	if (!p)
		return -ENOMEM;

	ret = lstat(p, &s);
	if (!ret) {
		ret = -EEXIST;
		goto out;
	}

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENODEV;
		goto out;
	}
	fsdrv = fsdev->driver;

	if (fsdrv->symlink) {
		ret = fsdrv->symlink(&fsdev->dev, pathname, p);
	} else {
		ret = -EPERM;
	}

out:
	free(freep);
	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(symlink);

static int fs_match(struct device_d *dev, struct driver_d *drv)
{
	return strcmp(dev->name, drv->name) ? -1 : 0;
}

static int fs_probe(struct device_d *dev)
{
	struct fs_device_d *fsdev = dev_to_fs_device(dev);
	struct fs_driver_d *fsdrv = dev_to_fs_driver(dev);
	int ret;

	ret = dev->driver->probe(dev);
	if (ret)
		return ret;

	fsdev->driver = fsdrv;

	list_add_tail(&fsdev->list, &fs_device_list);

	if (!fs_dev_root)
		fs_dev_root = fsdev;

	return 0;
}

static void fs_remove(struct device_d *dev)
{
	struct fs_device_d *fsdev = dev_to_fs_device(dev);

	if (fsdev->dev.driver) {
		dev->driver->remove(dev);
		list_del(&fsdev->list);
	}

	free(fsdev->path);

	if (fsdev == fs_dev_root)
		fs_dev_root = NULL;

	free(fsdev->backingstore);
	free(fsdev);
}

struct bus_type fs_bus = {
	.name = "fs",
	.match = fs_match,
	.probe = fs_probe,
	.remove = fs_remove,
};

static int fs_bus_init(void)
{
	return bus_register(&fs_bus);
}
pure_initcall(fs_bus_init);

int register_fs_driver(struct fs_driver_d *fsdrv)
{
	fsdrv->drv.bus = &fs_bus;
	register_driver(&fsdrv->drv);

	return 0;
}
EXPORT_SYMBOL(register_fs_driver);

static const char *detect_fs(const char *filename)
{
	enum filetype type = file_name_detect_type(filename);
	struct driver_d *drv;
	struct fs_driver_d *fdrv;

	if (type == filetype_unknown)
		return NULL;

	for_each_driver(drv) {
		if (drv->bus != &fs_bus)
			continue;

		fdrv = drv_to_fs_driver(drv);

		if (type == fdrv->type)
			return drv->name;
	}

	return NULL;
}

/*
 * Mount a device to a directory.
 * We do this by registering a new device on which the filesystem
 * driver will match.
 */
int mount(const char *device, const char *fsname, const char *_path)
{
	struct fs_device_d *fsdev;
	int ret;
	char *path = normalise_path(_path);

	debug("mount: %s on %s type %s\n", device, path, fsname);

	if (fs_dev_root) {
		fsdev = get_fsdevice_by_path(path);
		if (fsdev != fs_dev_root) {
			printf("sorry, no nested mounts\n");
			ret = -EBUSY;
			goto err_free_path;
		}
		ret = path_check_prereq(path, S_IFDIR);
		if (ret)
			goto err_free_path;
	} else {
		/* no mtab, so we only allow to mount on '/' */
		if (*path != '/' || *(path + 1)) {
			ret = -ENOTDIR;
			goto err_free_path;
		}
	}

	if (!fsname)
		fsname = detect_fs(device);

	if (!fsname)
		return -ENOENT;

	fsdev = xzalloc(sizeof(struct fs_device_d));
	fsdev->backingstore = xstrdup(device);
	safe_strncpy(fsdev->dev.name, fsname, MAX_DRIVER_NAME);
	fsdev->dev.id = get_free_deviceid(fsdev->dev.name);
	fsdev->path = xstrdup(path);
	fsdev->dev.bus = &fs_bus;

	if (!strncmp(device, "/dev/", 5))
		fsdev->cdev = cdev_by_name(device + 5);

	if (fsdev->cdev) {
		fsdev->dev.parent = fsdev->cdev->dev;
		fsdev->parent_device = fsdev->cdev->dev;
	}

	ret = register_device(&fsdev->dev);
	if (ret)
		goto err_register;

	if (!fsdev->dev.driver) {
		/*
		 * Driver didn't accept the device or no driver for this
		 * device. Bail out
		 */
		ret = -EINVAL;
		goto err_no_driver;
	}

	return 0;

err_no_driver:
	unregister_device(&fsdev->dev);
err_register:
	fs_remove(&fsdev->dev);
err_free_path:
	free(path);

	errno = -ret;

	return ret;
}
EXPORT_SYMBOL(mount);

int umount(const char *pathname)
{
	struct fs_device_d *fsdev = NULL, *f;
	char *p = normalise_path(pathname);

	for_each_fs_device(f) {
		if (!strcmp(p, f->path)) {
			fsdev = f;
			break;
		}
	}

	free(p);

	if (f == fs_dev_root && !list_is_singular(&fs_device_list)) {
		errno = EBUSY;
		return -EBUSY;
	}

	if (!fsdev) {
		errno = EFAULT;
		return -EFAULT;
	}

	unregister_device(&fsdev->dev);

	return 0;
}
EXPORT_SYMBOL(umount);

DIR *opendir(const char *pathname)
{
	DIR *dir = NULL;
	struct fs_device_d *fsdev;
	struct fs_driver_d *fsdrv;
	char *p = normalise_path(pathname);
	char *freep = p;
	int ret;

	ret = path_check_prereq(pathname, S_IFDIR);
	if (ret)
		goto out;

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENOENT;
		goto out;
	}
	fsdrv = fsdev->driver;

	debug("opendir: fsdrv: %p\n",fsdrv);

	dir = fsdrv->opendir(&fsdev->dev, p);
	if (dir) {
		dir->dev = &fsdev->dev;
		dir->fsdrv = fsdrv;
	} else {
		/*
		 * FIXME: The fs drivers should return ERR_PTR here so that
		 * we are able to forward the error
		 */
		ret = -EINVAL;
	}

out:
	free(freep);

	if (ret)
		errno = -ret;

	return dir;
}
EXPORT_SYMBOL(opendir);

struct dirent *readdir(DIR *dir)
{
	struct dirent *ent;

	if (!dir)
		return NULL;

	ent = dir->fsdrv->readdir(dir->dev, dir);

	if (!ent)
		errno = EBADF;

	return ent;
}
EXPORT_SYMBOL(readdir);

int closedir(DIR *dir)
{
	int ret;

	if (!dir) {
		errno = EBADF;
		return -EBADF;
	}

	ret = dir->fsdrv->closedir(dir->dev, dir);
	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(closedir);

int stat(const char *filename, struct stat *s)
{
	char *f;

	f = realfile(filename, s);
	if (IS_ERR(f))
		return PTR_ERR(f);

	free(f);
	return 0;
}
EXPORT_SYMBOL(stat);

int lstat(const char *filename, struct stat *s)
{
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	struct fs_device_d *fsdev;
	char *f = normalise_path(filename);
	char *freep = f;
	int ret;

	automount_mount(f, 1);

	memset(s, 0, sizeof(struct stat));

	fsdev = get_fsdevice_by_path(f);
	if (!fsdev) {
		ret = -ENOENT;
		goto out;
	}

	if (fsdev != fs_dev_root && strcmp(f, fsdev->path)) {
		f += strlen(fsdev->path);
		dev = &fsdev->dev;
	} else
		dev = &fs_dev_root->dev;

	fsdrv = dev_to_fs_driver(dev);

	if (*f == 0)
		f = "/";

	ret = fsdrv->stat(dev, f, s);
out:
	free(freep);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(lstat);

int mkdir (const char *pathname, mode_t mode)
{
	struct fs_driver_d *fsdrv;
	struct fs_device_d *fsdev;
	char *p = normalise_path(pathname);
	char *freep = p;
	int ret;

	ret = parent_check_directory(p);
	if (ret)
		goto out;

	ret = path_check_prereq(pathname, S_UB_DOES_NOT_EXIST);
	if (ret)
		goto out;

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENOENT;
		goto out;
	}
	fsdrv = fsdev->driver;

	if (fsdrv->mkdir)
		ret = fsdrv->mkdir(&fsdev->dev, p);
	else
		ret = -EROFS;
out:
	free(freep);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(mkdir);

int rmdir (const char *pathname)
{
	struct fs_driver_d *fsdrv;
	struct fs_device_d *fsdev;
	char *p = normalise_path(pathname);
	char *freep = p;
	int ret;

	ret = path_check_prereq(pathname, S_IFLNK);
	if (!ret) {
		ret = -ENOTDIR;
		goto out;
	}

	ret = path_check_prereq(pathname, S_IFDIR | S_UB_IS_EMPTY);
	if (ret)
		goto out;

	fsdev = get_fs_device_and_root_path(&p);
	if (!fsdev) {
		ret = -ENODEV;
		goto out;
	}
	fsdrv = fsdev->driver;

	if (fsdrv->rmdir)
		ret = fsdrv->rmdir(&fsdev->dev, p);
	else
		ret = -EROFS;
out:
	free(freep);

	if (ret)
		errno = -ret;

	return ret;
}
EXPORT_SYMBOL(rmdir);

static void memcpy_sz(void *_dst, const void *_src, ulong count, ulong rwsize)
{
	ulong dst = (ulong)_dst;
	ulong src = (ulong)_src;

	/* no rwsize specification given. Do whatever memcpy likes best */
	if (!rwsize) {
		memcpy(_dst, _src, count);
		return;
	}

	rwsize = rwsize >> O_RWSIZE_SHIFT;

	count /= rwsize;

	while (count-- > 0) {
		switch (rwsize) {
		case 1:
			*((u_char *)dst) = *((u_char *)src);
			break;
		case 2:
			*((ushort *)dst) = *((ushort *)src);
			break;
		case 4:
			*((ulong  *)dst) = *((ulong  *)src);
			break;
		}
		dst += rwsize;
		src += rwsize;
	}
}

ssize_t mem_read(struct cdev *cdev, void *buf, size_t count, loff_t offset, ulong flags)
{
	ulong size;
	struct device_d *dev;

	if (!cdev->dev || cdev->dev->num_resources < 1)
		return -1;
	dev = cdev->dev;

	size = min((resource_size_t)count,
			resource_size(&dev->resource[0]) -
			(resource_size_t)offset);
	memcpy_sz(buf, dev_get_mem_region(dev, 0) + offset, size, flags & O_RWSIZE_MASK);
	return size;
}
EXPORT_SYMBOL(mem_read);

ssize_t mem_write(struct cdev *cdev, const void *buf, size_t count, loff_t offset, ulong flags)
{
	ulong size;
	struct device_d *dev;

	if (!cdev->dev || cdev->dev->num_resources < 1)
		return -1;
	dev = cdev->dev;

	size = min((resource_size_t)count,
			resource_size(&dev->resource[0]) -
			(resource_size_t)offset);
	memcpy_sz(dev_get_mem_region(dev, 0) + offset, buf, size, flags & O_RWSIZE_MASK);
	return size;
}
EXPORT_SYMBOL(mem_write);
