/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 * Copyright (c) 2022 Bruno Herrera
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <assert.h>
#include <string.h>

#include <dev/virtio.h>
#include <dev/virtio/virtio_ring.h>
#include <kernel/thread.h>
#include <lk/compiler.h>
#include <lk/err.h>
#include <lk/init.h>
#include <lk/pow2.h>
#include <lk/trace.h>
#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#include "virtio_priv.h"

#if WITH_DEV_VIRTIO_BLOCK
#include <dev/virtio/block.h>
#endif
#if WITH_DEV_VIRTIO_NET
#include <dev/virtio/net.h>
#endif
#if WITH_DEV_VIRTIO_GPU
#include <dev/virtio/gpu.h>
#endif

#define LOCAL_TRACE 0

static inline void vring_init(struct vring *vr, unsigned int num, void *p, unsigned long align) {
  vr->num = num;
  vr->num_mask = (1 << log2_uint(num)) - 1;
  vr->free_list = 0xffff;
  vr->free_count = 0;
  vr->last_used = 0;
  vr->desc = p;
  vr->avail = p + num * sizeof(struct vring_desc);
  vr->used = (void *)(((unsigned long)&vr->avail->ring[num] + sizeof(uint16_t) + align - 1) &
                      ~(align - 1));
}

static inline unsigned vring_size(unsigned int num, unsigned long align) {
  return ((sizeof(struct vring_desc) * num + sizeof(uint16_t) * (3 + num) + align - 1) &
          ~(align - 1)) +
         sizeof(uint16_t) * 3 + sizeof(struct vring_used_elem) * num;
}

/* The following is used with USED_EVENT_IDX and AVAIL_EVENT_IDX */
/* Assuming a given event_idx value from the other size, if
 * we have just incremented index from old to new_idx,
 * should we trigger an event? */
static inline int vring_need_event(uint16_t event_idx, uint16_t new_idx, uint16_t old) {
  /* Note: Xen has similar logic for notification hold-off
   * in include/xen/interface/io/ring.h with req_event and req_prod
   * corresponding to event_idx + 1 and new_idx respectively.
   * Note also that req_event and req_prod in Xen start at 1,
   * event indexes in virtio start at 0. */
  return (uint16_t)(new_idx - event_idx - 1) < (uint16_t)(new_idx - old);
}

void virtio_free_desc(struct virtio_device *dev, uint ring_index, uint16_t desc_index) {
  LTRACEF("dev %p ring %u index %u free_count %u\n", dev, ring_index, desc_index,
          dev->ring[ring_index].free_count);
  dev->ring[ring_index].desc[desc_index].next = dev->ring[ring_index].free_list;
  dev->ring[ring_index].free_list = desc_index;
  dev->ring[ring_index].free_count++;
}

uint16_t virtio_alloc_desc(struct virtio_device *dev, uint ring_index) {
  if (dev->ring[ring_index].free_count == 0)
    return 0xffff;

  DEBUG_ASSERT(dev->ring[ring_index].free_list != 0xffff);

  uint16_t i = dev->ring[ring_index].free_list;
  struct vring_desc *desc = &dev->ring[ring_index].desc[i];
  dev->ring[ring_index].free_list = desc->next;

  dev->ring[ring_index].free_count--;

  return i;
}

struct vring_desc *virtio_alloc_desc_chain(struct virtio_device *dev, uint ring_index, size_t count,
                                           uint16_t *start_index) {
  if (dev->ring[ring_index].free_count < count)
    return NULL;

  /* start popping entries off the chain */
  struct vring_desc *last = 0;
  uint16_t last_index = 0;
  while (count > 0) {
    uint16_t i = dev->ring[ring_index].free_list;
    struct vring_desc *desc = &dev->ring[ring_index].desc[i];

    dev->ring[ring_index].free_list = desc->next;
    dev->ring[ring_index].free_count--;

    if (last) {
      desc->flags = VRING_DESC_F_NEXT;
      desc->next = last_index;
    } else {
      // first one
      desc->flags = 0;
      desc->next = 0;
    }
    last = desc;
    last_index = i;
    count--;
  }

  if (start_index)
    *start_index = last_index;

  return last;
}

void virtio_submit_chain(struct virtio_device *dev, uint ring_index, uint16_t desc_index) {
  LTRACEF("dev %p, ring %u, desc %u\n", dev, ring_index, desc_index);

  /* add the chain to the available list */
  struct vring_avail *avail = dev->ring[ring_index].avail;

  avail->ring[avail->idx & dev->ring[ring_index].num_mask] = desc_index;
  mb();
  avail->idx++;

#if LOCAL_TRACE
  hexdump(avail, 16);
#endif
}

status_t virtio_alloc_ring(struct virtio_device *dev, uint index, uint16_t len) {
  LTRACEF("dev %p, index %u, len %u\n", dev, index, len);

  DEBUG_ASSERT(dev);
  DEBUG_ASSERT(len > 0 && ispow2(len));
  DEBUG_ASSERT(index < MAX_VIRTIO_RINGS);

  if (len == 0 || !ispow2(len))
    return ERR_INVALID_ARGS;

  struct vring *ring = &dev->ring[index];

  /* allocate a ring */
  size_t size = vring_size(len, PAGE_SIZE);
  LTRACEF("need %zu bytes\n", size);

#if WITH_KERNEL_VM
  void *vptr;
  status_t err = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "virtio_ring", size, &vptr, 0, 0,
                                      ARCH_MMU_FLAG_UNCACHED_DEVICE);
  if (err < 0)
    return ERR_NO_MEMORY;

  LTRACEF("allocated virtio_ring at va %p\n", vptr);

  /* compute the physical address */
  paddr_t pa;
  pa = vaddr_to_paddr(vptr);
  if (pa == 0) {
    return ERR_NO_MEMORY;
  }

  LTRACEF("virtio_ring at pa 0x%lx\n", pa);
#else
  void *vptr = memalign(PAGE_SIZE, size);
  if (!vptr)
    return ERR_NO_MEMORY;

  LTRACEF("ptr %p\n", vptr);
  memset(vptr, 0, size);

  /* compute the physical address */
  paddr_t pa = (paddr_t)vptr;
#endif

  /* initialize the ring */
  vring_init(ring, len, vptr, PAGE_SIZE);
  dev->ring[index].free_list = 0xffff;
  dev->ring[index].free_count = 0;

  /* add all the descriptors to the free list */
  for (uint i = 0; i < len; i++) {
    virtio_free_desc(dev, index, i);
  }

  paddr_t pa_avail = pa + ((uintptr_t)dev->ring[index].avail - (uintptr_t)dev->ring[index].desc);
  paddr_t pa_used = pa + ((uintptr_t)dev->ring[index].used - (uintptr_t)dev->ring[index].desc);

  /* register the ring with the device */
  virtio_set_ring(dev, index, len, pa, pa_avail, pa_used);

  /* mark the ring active */
  dev->active_rings_bitmap |= (1 << index);

  return NO_ERROR;
}

void virtio_status_acknowledge_driver(struct virtio_device *dev) {
  uint8_t status = dev->config->get_status(dev);
  status |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
  dev->config->set_status(dev, status);
}

void virtio_status_driver_ok(struct virtio_device *dev) {
  uint8_t status = dev->config->get_status(dev);
  status |= VIRTIO_STATUS_DRIVER_OK;
  dev->config->set_status(dev, status);
}

void virtio_set_guest_features(struct virtio_device *dev, uint32_t features) {
  dev->config->set_guest_features(dev, features);
}

static void virtio_init(uint level) {}

void virtio_dump_desc(const struct vring_desc *desc) {
  printf("vring descriptor %p\n", desc);
  printf("\taddr  0x%llx\n", desc->addr);
  printf("\tlen   0x%x\n", desc->len);
  printf("\tflags 0x%hhx\n", desc->flags);
  printf("\tnext  0x%hhx\n", desc->next);
}

LK_INIT_HOOK(virtio, &virtio_init, LK_INIT_LEVEL_THREADING);
