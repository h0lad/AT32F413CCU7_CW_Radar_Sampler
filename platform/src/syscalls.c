// SPDX-License-Identifier: AGPL-3.0-or-later
#include <stddef.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

extern char end;
extern char _estack;

int _close(int file)
{
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

off_t _lseek(int file, off_t ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;
  errno = ENOSYS;
  return -1;
}

void *_sbrk(ptrdiff_t increment)
{
  static char *heap_end;
  char *previous_heap_end;

  if(heap_end == 0)
  {
    heap_end = &end;
  }

  previous_heap_end = heap_end;
  if((heap_end + increment) > (&_estack - 1024))
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  heap_end += increment;
  return previous_heap_end;
}

void _exit(int status)
{
  (void)status;
  while(1)
  {
  }
}
