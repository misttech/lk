/*
 * Copyright (c) 2025 Mist Tecnologia Ltda
 * Copyright (c) 2022 Bruno Herrera
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <assert.h>
#include <inttypes.h>
#include <lib/ddk/hw/arch_ops.h>
#include <lib/stdcompat/bit.h>
#include <string.h>
#include <trace.h>
#include <zircon/compiler.h>

#include <dev/virtio.h>
#include <fbl/algorithm.h>
#include <lk/compiler.h>
#include <lk/err.h>
#include <lk/pow2.h>
#include <lk/types.h>
#include <virtio/virtio.h>
#include <virtio/virtio_ring.h>
#include <vm/vm_aspace.h>

#include <ktl/enforce.h>

#define LOCAL_TRACE 0

void virtio_dump_desc(const struct vring_desc *desc) {
  printf("vring descriptor %p: ", desc);
  printf("[addr=%#" PRIx64 ", ", desc->addr);
  printf("len=%d, ", desc->len);
  printf("flags=%#04hx, ", desc->flags);
  printf("next=%#04hx]\n", desc->next);
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

struct vring_desc *virtio_alloc_desc_chain(struct virtio_device *dev, uint16_t ring_index,
                                           size_t count, uint16_t *start_index) {
  if (dev->ring[ring_index].free_count < count) {
    return NULL;
  }

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

  if (start_index) {
    *start_index = last_index;
  }
  return last;
}

void virtio_submit_chain(struct virtio_device *dev, uint16_t ring_index, uint16_t desc_index) {
  LTRACEF("dev %p, ring %u, desc %u\n", dev, ring_index, desc_index);

  /* add the chain to the available list */
  struct vring_avail *avail = dev->ring[ring_index].avail;

  avail->ring[avail->idx & dev->ring[ring_index].num_mask] = desc_index;
  // Write memory barrier before updating avail->idx; updates to the descriptor ring must be
  // visible before an updated avail->idx.
  hw_wmb();
  avail->idx++;

#if LOCAL_TRACE
  hexdump(avail, 16);
#endif
}

void virtio_kick(struct virtio_device *dev, uint16_t ring_index) {
  TRACEF("entry\n");
  // Write memory barrier before notifying the device. Updates to avail->idx must be visible
  // before the device sees the wakeup notification (so it processes the latest descriptors).
  hw_mb();
  dev->config->kick(dev, ring_index);
}

void virtio_set_ring(struct virtio_device *dev, uint16_t index, uint16_t count, paddr_t pa_desc,
                     paddr_t pa_avail, paddr_t pa_used) {
  dev->config->set_ring(dev, index, count, pa_desc, pa_avail, pa_used);
}

status_t virtio_alloc_ring(struct virtio_device *dev, uint16_t index, uint16_t count) {
  LTRACEF("dev %p, index %u, count %u\n", dev, index, count);

  // check that count is a power of 2
  if (!ktl::has_single_bit(count)) {
    dprintf(CRITICAL, "ring count: %u is not a power of 2\n", count);
    return ZX_ERR_INVALID_ARGS;
  }

  struct vring *ring = &dev->ring[index];

  // allocate a ring
  const size_t vring_required_size = vring_size(count, PAGE_SIZE);

  // DMA buffer size must be multiples of page size. Round up to the nearest
  // page size.
  const size_t dma_buffer_size = fbl::round_up(vring_required_size, static_cast<size_t>(PAGE_SIZE));
  LTRACEF("need %zu bytes\n", dma_buffer_size);

  void *vptr;
  zx_status_t status = VmAspace::kernel_aspace()->AllocContiguous(
      "virtio_ring", vring_required_size, &vptr, 0, VmAspace::VMM_FLAG_COMMIT,
      ARCH_MMU_FLAG_PERM_READ | ARCH_MMU_FLAG_PERM_WRITE);
  if (status != ZX_OK) {
    dprintf(CRITICAL, "failed to allocate ring buffer of size %zu: %d\n", dma_buffer_size, status);
    return status;
  }

  /* compute the physical address */
  paddr_t pa = vaddr_to_paddr(vptr);

  LTRACEF("allocated vring at %p, physical address %#" PRIxPTR "\n", vptr, pa);

  /* initialize the ring */
  vring_init(ring, count, vptr, PAGE_SIZE);

  dev->ring[index].free_list = 0xffff;
  dev->ring[index].free_count = 0;

  /* add all the descriptors to the free list */
  for (uint16_t i = 0; i < count; i++) {
    virtio_free_desc(dev, index, i);
  }

  /* register the ring with the device */
  paddr_t pa_avail = pa + ((uintptr_t)dev->ring[index].avail - (uintptr_t)dev->ring[index].desc);
  paddr_t pa_used = pa + ((uintptr_t)dev->ring[index].used - (uintptr_t)dev->ring[index].desc);
  virtio_set_ring(dev, index, count, pa, pa_avail, pa_used);

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
