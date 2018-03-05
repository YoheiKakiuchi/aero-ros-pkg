#ifndef __CONTROLLER_SHM_H__
#define __CONTROLLER_SHM_H__
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <stdio.h>

#include "shm_struct.h"
/* extern void *set_shared_memory(key_t key, size_t size); */

static void *set_shared_memory(key_t _key, size_t _size)
{
  int shm_id;
  void *ptr;
  int err;
  // First, try to allocate more memory than needed.
  // If this is the first shmget after reboot or
  // valid size of memory is already allocated,
  // shmget will succeed.
  // If the size of memory allocated is less than
  // _size*2,  shmget will fail.
  // e.g. Change the servo_shm.h then _size may increase.
  size_t size = _size * 2;
  key_t key = _key;
  shm_id=shmget(key, size, 0666|IPC_CREAT);
  err = errno;
  if(shm_id==-1 && err == EINVAL) {
    // if fail, retry with _size
    size = _size;
    shm_id=shmget(key, size, 0666|IPC_CREAT);
    err = errno;
  }
  if(shm_id==-1) {
    fprintf(stderr, "shmget failed, key=%d, size=%d, errno=%d\n", key, size, err);
    return NULL;
  }
  ptr=(struct shared_data *)shmat(shm_id, (void *)0, 0);
  if(ptr==(void *)-1) {
    int err=errno;
    fprintf(stderr, "shmat failed, key=%d, size=%d, shm_id=%d, errno=%d\n", key, size, shm_id, err);
    return NULL;
  }
  //fprintf(stderr, "shmget ok, size=%d\n", size);
  return ptr;
}

#endif
