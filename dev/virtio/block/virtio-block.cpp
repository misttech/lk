/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <assert.h>
#include <inttypes.h>
#include <lib/bio.h>
#include <stdlib.h>

#include <dev/virtio/block.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <kernel/thread.h>
#include <lk/compiler.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/list.h>
#include <lk/trace.h>
#include <vm/pmm.h>

#define LOCAL_TRACE 2

struct virtio_blk_config {
  uint64_t capacity;
  uint32_t size_max;
  uint32_t seg_max;
  struct virtio_blk_geometry {
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors;
  } geometry;
  uint32_t blk_size;
  struct virtio_blk_topology {
    uint8_t physical_block_exp;
    uint8_t alignment_offset;
    uint16_t min_io_size;
    uint32_t opt_io_size;
  } topology;
  uint8_t writeback;
  uint8_t unused[3];
  uint32_t max_discard_sectors;
  uint32_t max_discard_seq;
  uint32_t discard_sector_alignment;
  uint32_t max_write_zeroes_sectors;
  uint32_t max_write_zeroes_seq;
  uint8_t write_zeros_may_unmap;
  uint8_t unused1[3];
  uint32_t max_secure_erase_sectors;
  uint32_t max_secure_erase_seg;
  uint32_t secure_erase_sector_alignment;
  struct virtio_blk_zoned_characteristics {
    uint32_t zone_sectors;
    uint32_t max_open_zones;
    uint32_t max_active_zones;
    uint32_t max_append_sectors;
    uint32_t write_granularity;
    uint8_t model;
    uint8_t unused2[3];
  } zoned;
};
static_assert(sizeof(struct virtio_blk_config) == 96);

struct virtio_blk_req {
  uint32_t type;
  uint32_t ioprio;  // v1.3 says this is 'reserved'
  uint64_t sector;
};
static_assert(sizeof(struct virtio_blk_req) == 16);

struct virtio_blk_discard_write_zeroes {
  uint64_t sector;
  uint32_t num_sectors;
  struct {
    uint32_t unmap : 1;
    uint32_t reserved : 31;
  } flags;
};
static_assert(sizeof(struct virtio_blk_req) == 16);

#define VIRTIO_BLK_F_BARRIER (1 << 0)  // legacy
#define VIRTIO_BLK_F_SIZE_MAX (1 << 1)
#define VIRTIO_BLK_F_SEG_MAX (1 << 2)
#define VIRTIO_BLK_F_GEOMETRY (1 << 4)
#define VIRTIO_BLK_F_RO (1 << 5)
#define VIRTIO_BLK_F_BLK_SIZE (1 << 6)
#define VIRTIO_BLK_F_SCSI (1 << 7)  // legacy
#define VIRTIO_BLK_F_FLUSH (1 << 9)
#define VIRTIO_BLK_F_TOPOLOGY (1 << 10)
#define VIRTIO_BLK_F_CONFIG_WCE (1 << 11)
#define VIRTIO_BLK_F_DISCARD (1 << 13)
#define VIRTIO_BLK_F_WRITE_ZEROES (1 << 14)
#define VIRTIO_BLK_F_LIFETIME (1 << 15)
#define VIRTIO_BLK_F_SECURE_ERASE (1 << 16)
#define VIRTIO_BLK_F_ZONED (1 << 17)

#define VIRTIO_BLK_T_IN 0
#define VIRTIO_BLK_T_OUT 1
#define VIRTIO_BLK_T_FLUSH 4
#define VIRTIO_BLK_T_GET_ID 8
#define VIRTIO_BLK_T_GET_LIFETIME 10
#define VIRTIO_BLK_T_DISCARD 11
#define VIRTIO_BLK_T_WRITE_ZEROES 13
#define VIRTIO_BLK_T_SECURE_ERASE 13

#define VIRTIO_BLK_S_OK 0
#define VIRTIO_BLK_S_IOERR 1
#define VIRTIO_BLK_S_UNSUPP 2

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring,
                                                            const struct vring_used_elem *e);
static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count);
static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block,
                                       uint count);

struct virtio_block_dev : public bdev_t {
  struct virtio_device *dev;

  /* our negotiated guest features */
  uint32_t guest_features;

  /* one blk_req structure for io, not crossing a page boundary */
  struct virtio_blk_req *blk_req = nullptr;
  paddr_t blk_req_phys = 0;

  /* one uint8_t response word */
  zx_paddr_t blk_response_phys = 0;
  uint8_t *blk_response = nullptr;

  /* non-standard layout members at the end */
  DECLARE_MUTEX(virtio_block_dev) lock;
  AutounsignalEvent io_event;
};

static void dump_feature_bits(const char *name, uint32_t feature) {
  printf("virtio-block %s features (%#x):", name, feature);
  if (feature & VIRTIO_BLK_F_BARRIER)
    printf(" BARRIER");
  if (feature & VIRTIO_BLK_F_SIZE_MAX)
    printf(" SIZE_MAX");
  if (feature & VIRTIO_BLK_F_SEG_MAX)
    printf(" SEG_MAX");
  if (feature & VIRTIO_BLK_F_GEOMETRY)
    printf(" GEOMETRY");
  if (feature & VIRTIO_BLK_F_RO)
    printf(" RO");
  if (feature & VIRTIO_BLK_F_BLK_SIZE)
    printf(" BLK_SIZE");
  if (feature & VIRTIO_BLK_F_SCSI)
    printf(" SCSI");
  if (feature & VIRTIO_BLK_F_FLUSH)
    printf(" FLUSH");
  if (feature & VIRTIO_BLK_F_TOPOLOGY)
    printf(" TOPOLOGY");
  if (feature & VIRTIO_BLK_F_CONFIG_WCE)
    printf(" CONFIG_WCE");
  if (feature & VIRTIO_BLK_F_DISCARD)
    printf(" DISCARD");
  if (feature & VIRTIO_BLK_F_WRITE_ZEROES)
    printf(" WRITE_ZEROES");
  if (feature & VIRTIO_BLK_F_LIFETIME)
    printf(" LIFETIME");
  if (feature & VIRTIO_BLK_F_SECURE_ERASE)
    printf(" SECURE_ERASE");
  if (feature & VIRTIO_BLK_F_ZONED)
    printf(" ZONED");
  printf("\n");
}

status_t virtio_block_init(struct virtio_device *dev, uint32_t host_features) {
  virtio_reset_device(dev);

  volatile struct virtio_blk_config *config = (struct virtio_blk_config *)dev->config_ptr;

  // TODO(cja): The blk_size provided in the device configuration is only
  // populated if a specific feature bit has been negotiated during
  // initialization, otherwise it is 0, at least in Virtio 0.9.5. Use 512
  // as a default as a stopgap for now until proper feature negotiation
  // is supported.
  if (config->blk_size == 0) {
    config->blk_size = 512;
  }

  LTRACEF("capacity %" PRIx64 "\n", config->capacity);
  LTRACEF("size_max %#x\n", config->size_max);
  LTRACEF("seg_max  %#x\n", config->seg_max);
  LTRACEF("blk_size %#x\n", config->blk_size);

  /* ack and set the driver status bit */
  virtio_status_acknowledge_driver(dev);

  /* allocate a new block device */
  fbl::AllocChecker ac;
  struct virtio_block_dev *bdev = new (&ac) struct virtio_block_dev;
  if (!ac.check()) {
    return ERR_NO_MEMORY;
  }

  bdev->dev = dev;
  dev->priv = bdev;
  bdev->blk_req = (struct virtio_blk_req *)memalign(sizeof(struct virtio_blk_req),
                                                    sizeof(struct virtio_blk_req));
  bdev->blk_req_phys = vaddr_to_paddr(bdev->blk_req);

  LTRACEF("allocated blk request at %p, physical address %#" PRIxPTR "\n", bdev->blk_req,
          bdev->blk_req_phys);

  // Responses are 32 words at the end of the allocated block.
  bdev->blk_response_phys = bdev->blk_req_phys + sizeof(struct virtio_blk_req) * kBlkReqCount;
  bdev->blk_response = reinterpret_cast<uint8_t *>(
      (reinterpret_cast<uintptr_t>(bdev->blk_req) + sizeof(struct virtio_blk_req) * kBlkReqCount));

  LTRACEF("allocated blk responses at %p, physical address %#" PRIxPTR "\n", bdev->blk_response,
          bdev->blk_response_phys);

  /* check features bits and ack/nak them */
  bdev->guest_features = host_features;

  /* keep the features we understand or can tolerate */
  bdev->guest_features &= (VIRTIO_BLK_F_SIZE_MAX | VIRTIO_BLK_F_BLK_SIZE | VIRTIO_BLK_F_GEOMETRY |
                           VIRTIO_BLK_F_BLK_SIZE | VIRTIO_BLK_F_TOPOLOGY | VIRTIO_BLK_F_DISCARD |
                           VIRTIO_BLK_F_WRITE_ZEROES);
  virtio_set_guest_features(dev, bdev->guest_features);

  /* TODO: handle a RO feature */

  // Allocate the main vring.
  auto err = virtio_alloc_ring(dev, 0, kRingSize);
  if (err < 0) {
    dprintf(CRITICAL, "Failed to allocate vring: %d\n", err);
    return err;
  }

  /* set our irq handler */
  dev->irq_driver_callback = &virtio_block_irq_driver_callback;

  /* set DRIVER_OK */
  virtio_status_driver_ok(dev);

  printf("virtio-block found device of size %" PRIu64 "\n", config->capacity * config->blk_size);

  /* dump feature bits */
  dump_feature_bits("host", host_features);
  dump_feature_bits("guest", bdev->guest_features);
  printf("\tsize_max %u seg_max %u\n", config->size_max, config->seg_max);
  if (host_features & VIRTIO_BLK_F_GEOMETRY) {
    printf("\tgeometry: cyl %u head %u sector %u\n", config->geometry.cylinders,
           config->geometry.heads, config->geometry.sectors);
  }
  if (host_features & VIRTIO_BLK_F_BLK_SIZE) {
    printf("\tblock_size: %u\n", config->blk_size);
  }
  if (host_features & VIRTIO_BLK_F_TOPOLOGY) {
    printf("\ttopology: block exp %u alignment_offset %u min_io_size %u opt_io_size %u\n",
           config->topology.physical_block_exp, config->topology.alignment_offset,
           config->topology.min_io_size, config->topology.opt_io_size);
  }
  if (host_features & VIRTIO_BLK_F_DISCARD) {
    printf("\tdiscard: max sectors %u max sequence %u alignment %u\n", config->max_discard_sectors,
           config->max_discard_sectors, config->discard_sector_alignment);
  }
  if (host_features & VIRTIO_BLK_F_WRITE_ZEROES) {
    printf("\twrite zeroes: max sectors %u max sequence %u may unmap %u\n",
           config->max_write_zeroes_sectors, config->max_write_zeroes_seq,
           config->write_zeros_may_unmap);
  }

  /* construct the block device */
  static uint8_t found_index = 0;
  ktl::array<char, ZX_MAX_NAME_LEN> block_name{};
  snprintf(block_name.data(), block_name.size(), "virtio%u", found_index++);
  bio_initialize_bdev(bdev, block_name.data(), config->blk_size,
                      static_cast<bnum_t>(config->capacity), 0, nullptr, BIO_FLAGS_NONE);

  /* override our block device hooks */
  bdev->read_block = &virtio_bdev_read_block;
  bdev->write_block = &virtio_bdev_write_block;

  bio_register_device(bdev);

  return NO_ERROR;
}

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring,
                                                            const struct vring_used_elem *e) {
  struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

  LTRACEF("dev %p, ring %u, e %p, id %u, len %u\n", dev, ring, e, e->id, e->len);
  /* parse our descriptor chain, add back to the free queue */
  uint16_t i = static_cast<uint16_t>(e->id);
  for (;;) {
    int next;
    struct vring_desc *desc = virtio_desc_index_to_desc(dev, ring, i);

    // virtio_dump_desc(desc);

    if (desc->flags & VRING_DESC_F_NEXT) {
      next = desc->next;
    } else {
      /* end of chain */
      next = -1;
    }

    virtio_free_desc(dev, ring, i);

    if (next < 0) {
      break;
    }

    i = static_cast<uint16_t>(next);
  }

  /* signal our event */
  bdev->io_event.Signal();

  return INT_RESCHEDULE;
}

static ssize_t virtio_block_read_write(struct virtio_device *dev, void *buf, const off_t offset,
                                       const size_t len, const bool write) {
  struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

  uint16_t i;
  struct vring_desc *desc;

  LTRACEF("dev %p, buf %p, offset 0x%llx, len %zu\n", dev, buf, offset, len);

  Guard<Mutex> lock(&bdev->lock);

  /* set up the request */
  bdev->blk_req->type = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
  bdev->blk_req->ioprio = 0;
  bdev->blk_req->sector = offset / 512;
  LTRACEF("blk_req type %u ioprio %u sector %lu\n", bdev->blk_req->type, bdev->blk_req->ioprio,
          bdev->blk_req->sector);

  /* put together a transfer */
  desc = virtio_alloc_desc_chain(dev, 0, 3, &i);
  LTRACEF("after alloc chain desc %p, i %u\n", desc, i);

  // XXX not cache safe.
  // At the moment only tested on arm qemu, which doesn't emulate cache.

  /* set up the descriptor pointing to the head */
  desc->addr = bdev->blk_req_phys;
  desc->len = sizeof(struct virtio_blk_req);
  desc->flags |= VRING_DESC_F_NEXT;

  /* set up the descriptor pointing to the buffer */
  desc = virtio_desc_index_to_desc(dev, 0, desc->next);
  /* translate the first buffer */
  vaddr_t va = (vaddr_t)buf;
  paddr_t pa = vaddr_to_paddr((void *)va);
  desc->addr = (uint64_t)pa;
  /* desc->len is filled in below */
  desc->flags |= write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
  desc->flags |= VRING_DESC_F_NEXT;

  /* see if we need to add more descriptors due to scatter gather */
  paddr_t next_pa = PAGE_ALIGN(pa + 1);
  desc->len = static_cast<uint32_t>(ktl::min(static_cast<size_t>(next_pa - pa), len));
  LTRACEF("first descriptor va 0x%lx desc->addr 0x%lx desc->len %u\n", va, desc->addr, desc->len);

  size_t remaining_len = len;
  remaining_len -= desc->len;
  while (remaining_len > 0) {
    /* amount of source buffer handled by this iteration of the loop */
    size_t len_tohandle = ktl::min(remaining_len, static_cast<size_t>(PAGE_SIZE));

    /* translate the next page in the buffer */
    va = PAGE_ALIGN(va + 1);
    pa = vaddr_to_paddr((void *)va);
    LTRACEF("va now 0x%lx, pa 0x%lx, next_pa 0x%lx, remaining len %zu\n", va, pa, next_pa,
            remaining_len);

    /* is the new translated physical address contiguous to the last one? */
    if (next_pa == pa) {
      /* we can simply extend the previous descriptor by another page */
      LTRACEF("extending last one by %zu bytes\n", len_tohandle);
      desc->len += len_tohandle;
    } else {
      /* new physical page needed, allocate a new descriptor and start again */
      uint16_t next_i = virtio_alloc_desc(dev, 0);
      struct vring_desc *next_desc = virtio_desc_index_to_desc(dev, 0, next_i);
      DEBUG_ASSERT(next_desc);

      LTRACEF("doesn't extend, need new desc, allocated desc %i (%p)\n", next_i, next_desc);

      /* fill this descriptor in and put it after the last one but before the response descriptor */
      next_desc->addr = (uint64_t)pa;
      next_desc->len = static_cast<uint32_t>(len_tohandle);
      next_desc->flags =
          write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
      next_desc->flags |= VRING_DESC_F_NEXT;
      next_desc->next = desc->next;
      desc->next = next_i;

      desc = next_desc;
    }
    remaining_len -= len_tohandle;
    next_pa += PAGE_SIZE;
  }

  /* set up the descriptor pointing to the response */
  desc = virtio_desc_index_to_desc(dev, 0, desc->next);
  desc->addr = bdev->blk_response_phys;
  desc->len = 1;
  desc->flags = VRING_DESC_F_WRITE;

  /* submit the transfer */
  virtio_submit_chain(dev, 0, i);

  /* kick it off */
  virtio_kick(dev, 0);

  /* wait for the transfer to complete */
  bdev->io_event.Wait();

  LTRACEF("status 0x%hhx\n", bdev->blk_response[0]);

  /* TODO: handle transfer errors and return error */

  return len;
}

static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count) {
  struct virtio_block_dev *dev = reinterpret_cast<struct virtio_block_dev *>(bdev);

  LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

  ssize_t result =
      virtio_block_read_write(dev->dev, buf, (off_t)block * dev->block_size, count, false);
  return result;
}

static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block,
                                       uint count) {
  struct virtio_block_dev *dev = reinterpret_cast<struct virtio_block_dev *>(bdev);

  LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

  ssize_t result =
      virtio_block_read_write(dev->dev, (void *)buf, (off_t)block * dev->block_size, count, true);
  return result;
}
