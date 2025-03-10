/*
 * Copyright (c) 2025 Mist Tecnologia Ltda
 * Copyright (c) 2022 Bruno Herrera
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

#include <string.h>

#include <arch/ops.h>
#include <dev/bus/pci.h>
#include <dev/virtio.h>
#include <dev/virtio/block.h>
#include <fbl/alloc_checker.h>
#include <lk/cpp.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/init.h>
#include <lk/list.h>
#include <lk/trace.h>
#include <vm/vm_aspace.h>

#include "virtio/virtio.h"

#define LOCAL_TRACE 0

struct virtio_device_id {
  uint16_t id;
};

const virtio_device_id virtio_ids[] = {
    {VIRTIO_DEV_TYPE_T_NETWORK}, {VIRTIO_DEV_TYPE_T_BLOCK},     {VIRTIO_DEV_TYPE_T_BALLOON},
    {VIRTIO_DEV_TYPE_T_CONSOLE}, {VIRTIO_DEV_TYPE_T_SCSI_HOST}, {VIRTIO_DEV_TYPE_T_ENTROPY},
    {VIRTIO_DEV_TYPE_T_9P},
};

class virtio_pci_modern {
 public:
  virtio_pci_modern();

  ~virtio_pci_modern();

  status_t init_device(pci_location_t loc, const virtio_device_id *id);

 private:
  status_t virtio_pci_find_capabilities();

  // counter of configured deices
  static volatile int global_count_;
  int unit_ = 0;

  // configuration
  pci_location_t loc_ = {};
  const virtio_device_id *id_feat_ = nullptr;

  intptr_t notify_base_ = 0;
  volatile uint32_t *isr_status_ = nullptr;
  uintptr_t device_cfg_ = 0;
  volatile virtio_pci_common_cfg_t *common_cfg_ = nullptr;
  uint32_t notify_off_mul_;

  pci_bar_t bars_[6];
  void *mbar_[6] = {};
};

volatile int virtio_pci_modern::global_count_ = 0;

virtio_pci_modern::virtio_pci_modern() = default;
virtio_pci_modern::~virtio_pci_modern() {
  // TODO: free resources
}

status_t map_bar(int id, pci_bar_t *bar, void **ptr) {
  char name[32];

  LTRACEF("Request bar %d mapped to %p\n", id, ptr);

  if (*ptr) {
    return NO_ERROR;
  }

  snprintf(name, sizeof(name), "bar%d", id);
  // Set the name of the vmo for tracking
  // char name[32];
  // snprintf(name, sizeof(name), "pci-%02x:%02x.%1x-bar%u", loc_.bus, loc_.dev, loc_.func, id);

  zx_status_t res = VmAspace::kernel_aspace()->AllocPhysical(
      name, ktl::max<uint64_t>(bar->size, PAGE_SIZE), /* size */
      ptr,                                            /* returned virtual address */
      PAGE_SIZE_SHIFT,                                /* alignment log2 */
      bar->addr,                                      /* physical address */
      0,                                              /* vmm flags */
      ARCH_MMU_FLAG_UNCACHED_DEVICE | ARCH_MMU_FLAG_PERM_READ | ARCH_MMU_FLAG_PERM_WRITE);
  if (res != ZX_OK) {
    LTRACEF("failed to map bar %p\n", bar);
    return res;
  }

  LTRACEF("bar %d regs mapped to %p\n", id, ptr);

  return NO_ERROR;
}

status_t virtio_pci_read_cap(const pci_location_t loc, const pci_capability_node_t *entry,
                             virtio_pci_cap_t *cap) {
  status_t err;

  *cap = {};

  // TODO: handle endian swapping (if necessary)

  // define some helper routines to read config offsets in the proper unit
  uint32_t next_index;
  auto read_byte = [&]() -> uint8_t {
    uint8_t val;
    err = pci_read_config_byte(loc, next_index, &val);
    next_index++;
    if (err < 0)
      return 0;
    return val;
  };

  auto read_word = [&]() -> uint32_t {
    uint32_t val;
    err = pci_read_config_word(loc, next_index, &val);
    next_index += 4;
    if (err < 0)
      return 0;
    return val;
  };

  next_index = entry->config_offset;
  cap->cap_vndr = read_byte();
  if (err < 0)
    return err;
  cap->cap_next = read_byte();
  if (err < 0)
    return err;
  cap->cap_len = read_byte();
  if (err < 0)
    return err;
  cap->cfg_type = read_byte();
  if (err < 0)
    return err;
  cap->bar = read_byte();
  if (err < 0)
    return err;
  next_index += 3;  // padding[3];
  cap->offset = read_word();
  if (err < 0)
    return err;
  cap->length = read_word();
  if (err < 0)
    return err;

  DEBUG_ASSERT(next_index ==
               (entry->config_offset + 0x10));  // should have read this many bytes at this point

  return NO_ERROR;
}

constexpr uint8_t PCIE_CAP_ID_VENDOR = 0x09;
status_t virtio_pci_modern::virtio_pci_find_capabilities() {
  LTRACE_ENTRY;

  struct list_node caps;
  list_initialize(&caps);

  status_t err = pci_device_filter_capabilities(loc_, PCIE_CAP_ID_VENDOR, &caps);
  if (err == NO_ERROR) {
    pci_capability_node_t *c;
    list_for_every_entry (&caps, c, pci_capability_node_t, node) {
      LTRACEF("cap id %#x at offset %#x\n", c->id, c->config_offset);
      virtio_pci_cap_t cap;
      err = virtio_pci_read_cap(loc_, c, &cap);
      if (err != NO_ERROR) {
        LTRACEF("Failed to read PCI capabilities\n");
        return err;
      }

      switch (cap.cfg_type) {
        case VIRTIO_PCI_CAP_COMMON_CFG:
          LTRACEF("common cfg found in bar %u offset %#x\n", cap.bar, cap.offset);
          {
            map_bar(cap.bar, &bars_[cap.bar], &mbar_[cap.bar]);
            auto addr = reinterpret_cast<uintptr_t>(mbar_[cap.bar]) + cap.offset;
            common_cfg_ = reinterpret_cast<volatile virtio_pci_common_cfg_t *>(addr);
          }
          break;
        case VIRTIO_PCI_CAP_NOTIFY_CFG:
          // Virtio 1.0 section 4.1.4.4
          // notify_off_multiplier is a 32bit field following this capability
          LTRACEF("notify cfg found in bar %u offset %#x\n", cap.bar, cap.offset);
          pci_read_config_word(loc_, c->config_offset + sizeof(virtio_pci_cap_t), &notify_off_mul_);
          map_bar(cap.bar, &bars_[cap.bar], &mbar_[cap.bar]);
          notify_base_ = reinterpret_cast<uintptr_t>(mbar_[cap.bar]) + cap.offset;
          break;
        case VIRTIO_PCI_CAP_ISR_CFG:
          LTRACEF("isr cfg found in bar %u offset %#x\n", cap.bar, cap.offset);
          map_bar(cap.bar, &bars_[cap.bar], &mbar_[cap.bar]);
          // interrupt status is directly read from the register at this address
          isr_status_ = reinterpret_cast<volatile uint32_t *>(
              reinterpret_cast<uintptr_t>(mbar_[cap.bar]) + cap.offset);
          break;
        case VIRTIO_PCI_CAP_DEVICE_CFG:
          LTRACEF("device cfg found in bar %u offset %#x\n", cap.bar, cap.offset);
          map_bar(cap.bar, &bars_[cap.bar], &mbar_[cap.bar]);
          device_cfg_ = reinterpret_cast<uintptr_t>(mbar_[cap.bar]) + cap.offset;
          break;
        case VIRTIO_PCI_CAP_PCI_CFG:
          // We are not using this capability presently since we can map the
          // bars for direct memory access.
          break;
        case VIRTIO_PCI_CAP_SHARED_MEMORY_CFG:
          break;
      }
    }
  }

  // Ensure we found needed capabilities during parsing
  if (common_cfg_ == nullptr || isr_status_ == nullptr || device_cfg_ == 0 || notify_base_ == 0) {
    LTRACEF("Failed to bind, missing capabilities\n");
    return ZX_ERR_BAD_STATE;
  }

  LTRACEF("virtio: modern pci backend successfully initialized\n");
  return ZX_OK;
}

static void virtio_pci_scan(uint level) {
  LTRACE_ENTRY;

  // probe pci to find a device
  for (auto id : virtio_ids) {
    for (size_t i = 0;; i++) {
      pci_location_t loc;
      status_t err = pci_bus_mgr_find_device(&loc, id.id, VIRTIO_PCI_VENDOR_ID, i);
      if (err != NO_ERROR) {
        break;
      }

      // we maybe found one, create a new device and initialize it
      fbl::AllocChecker ac_;
      auto vpm = new (&ac_) virtio_pci_modern;
      if (!ac_.check()) {
        break;
      }
      err = vpm->init_device(loc, &id);
      if (err != NO_ERROR) {
        char str[14];
        printf("virtio_pci: device at %s failed to initialize\n", pci_loc_string(loc, str));
        delete vpm;
        continue;
      }
    }
  }
}

extern "C" {

struct virtio_pci_modern_device {
  struct virtio_device dev;

  intptr_t notify_base;
  volatile uint32_t *isr_status;
  uintptr_t device_cfg;
  volatile virtio_pci_common_cfg_t *common_cfg;
  uint32_t notify_off_mul;
};

#define to_virtio_pci_modern_device(_plat_dev) \
  containerof(_plat_dev, struct virtio_pci_modern_device, dev)

static void virtio_pci_modern_kick(struct virtio_device *dev, uint ring_index) {
  LTRACEF("dev %p, ring %u\n", dev, ring_index);
  struct virtio_pci_modern_device *pdev = to_virtio_pci_modern_device(dev);

  // Virtio 1.0 Section 4.1.4.4
  // The address to notify for a queue is calculated using information from
  // the notify_off_multiplier, the capability's base + offset, and the
  // selected queue's offset.
  //
  // For performance reasons, we assume that the selected queue's offset is
  // equal to the ring index.
  auto addr = pdev->notify_base + ring_index * pdev->notify_off_mul;
  auto ptr = reinterpret_cast<volatile uint16_t *>(addr);
  LTRACEF("kick %u addr %p\n", ring_index, ptr);
  *ptr = static_cast<uint16_t>(ring_index);
}

static void virtio_pci_modern_set_status(struct virtio_device *dev, uint8_t status) {
  struct virtio_pci_modern_device *pdev = to_virtio_pci_modern_device(dev);
  pdev->common_cfg->device_status = status;
}

static uint8_t virtio_pci_modern_get_status(struct virtio_device *dev) {
  struct virtio_pci_modern_device *pdev = to_virtio_pci_modern_device(dev);
  return pdev->common_cfg->device_status;
}

static void virtio_pci_modern_reset_device(struct virtio_device *vdev) {
  virtio_pci_modern_set_status(vdev, 0);
}

static void virtio_pci_modern_set_ring(struct virtio_device *dev, uint16_t index, uint16_t count,
                                       paddr_t pa_desc, paddr_t pa_avail, paddr_t pa_used) {
  LTRACEF("dev %p, ring %u\n", dev, index);
  struct virtio_pci_modern_device *pdev = to_virtio_pci_modern_device(dev);

  pdev->common_cfg->queue_select = index;
  pdev->common_cfg->queue_size = count;
  pdev->common_cfg->queue_desc = pa_desc;
  pdev->common_cfg->queue_avail = pa_avail;
  pdev->common_cfg->queue_used = pa_used;

  /*if (irq_mode() == PCI_IRQ_MODE_MSI_X) {
      uint16_t vector = 0;
      MmioWrite(&common_cfg_->config_msix_vector, PciBackend::kMsiConfigVector);
      MmioRead(&common_cfg_->config_msix_vector, &vector);
      if (vector != PciBackend::kMsiConfigVector) {
          zxlogf(ERROR, "MSI-X config vector in invalid state after write: %#x", vector);
          return ZX_ERR_BAD_STATE;
      }

      MmioWrite(&common_cfg_->queue_msix_vector, PciBackend::kMsiQueueVector);
      MmioRead(&common_cfg_->queue_msix_vector, &vector);
      if (vector != PciBackend::kMsiQueueVector) {
          zxlogf(ERROR, "MSI-X queue vector in invalid state after write: %#x", vector);
          return ZX_ERR_BAD_STATE;
      }
  }*/

  pdev->common_cfg->queue_enable = 1;

  // Assert that queue_notify_off is equal to the ring index.
  // uint16_t queue_notify_off;
  // FIXME (HERRERA) Review read/write direct from config structure (look fuchsia pci_modern.cc)
  if (pdev->common_cfg->queue_notify_off != index) {
    LTRACEF("Virtio queue notify setup failed\n");
    // return ZX_ERR_BAD_STATE;
  }
}

static void virtio_pci_set_guest_features(struct virtio_device *dev, uint32_t features) {
  printf("virtio_pci: virtio_pci_set_guest_features not implemented\n");
}

static const struct virtio_config_ops virtio_pci_modern_config_ops = {
    .reset = virtio_pci_modern_reset_device,
    .get_status = virtio_pci_modern_get_status,
    .set_status = virtio_pci_modern_set_status,
    .kick = virtio_pci_modern_kick,
    .set_ring = virtio_pci_modern_set_ring,
    .set_guest_features = virtio_pci_set_guest_features,
};

static void virtio_pci_modern_irq(void *arg) {
  struct virtio_pci_modern_device *pdev = (struct virtio_pci_modern_device *)arg;
  DEBUG_ASSERT(pdev);

  struct virtio_device *dev = &pdev->dev;
  DEBUG_ASSERT(dev);
  LTRACEF("dev %p, index %u\n", dev, dev->index);

  uint32_t irq_status = *pdev->isr_status;
  LTRACEF("status 0x%x\n", irq_status);

  // int ret = INT_NO_RESCHEDULE;
  if (irq_status & 0x1) { /* used ring update */
    /* cycle through all the active rings */
    for (uint r = 0; r < MAX_VIRTIO_RINGS; r++) {
      if ((dev->active_rings_bitmap & (1 << r)) == 0)
        continue;

      struct vring *ring = &dev->ring[r];
      LTRACEF("ring %u: used flags 0x%hx idx 0x%hx last_used %u\n", r, ring->used->flags,
              ring->used->idx, ring->last_used);

      uint cur_idx = ring->used->idx;
      for (uint i = ring->last_used; i != (cur_idx & ring->num_mask);
           i = (i + 1) & ring->num_mask) {
        LTRACEF("looking at idx %u\n", i);

        // process chain
        struct vring_used_elem *used_elem = &ring->used->ring[i];
        LTRACEF("id %u, len %u\n", used_elem->id, used_elem->len);

        DEBUG_ASSERT(dev->irq_driver_callback);
        // ret |= dev->irq_driver_callback(dev, r, used_elem);
        dev->irq_driver_callback(dev, r, used_elem);

        ring->last_used = (ring->last_used + 1) & ring->num_mask;
      }
    }
  }
  if (irq_status & 0x2) { /* config change */
    if (dev->config_change_callback) {
      // ret |= dev->config_change_callback(&pdev->dev);
      dev->config_change_callback(&pdev->dev);
    }
  }

  LTRACEF("exiting irq\n");

  // return static_cast<handler_return>(ret);
}
}

status_t virtio_pci_modern::init_device(pci_location_t loc, const virtio_device_id *id) {
  loc_ = loc;
  id_feat_ = id;
  char str[32];

  LTRACEF("pci location %s\n", pci_loc_string(loc_, str));

  status_t err = pci_bus_mgr_read_bars(loc_, bars_);
  if (err != NO_ERROR)
    return err;

  LTRACEF("virtio_pci BARS:\n");
  if (LOCAL_TRACE)
    pci_dump_bars(bars_, 6);

  if (!bars_[0].valid || bars_[0].addr == 0) {
    return ERR_NOT_FOUND;
  }

  // allocate a unit number
  unit_ = __atomic_fetch_add(&global_count_, 1, __ATOMIC_RELAXED);

  virtio_pci_find_capabilities();

  // Ensure we found needed capabilities during parsing
  if (common_cfg_ == nullptr || isr_status_ == nullptr || device_cfg_ == 0 || notify_base_ == 0) {
    LTRACEF("failed to bind, missing capabilities");
    return ERR_BAD_STATE;
  }

  pci_bus_mgr_enable_device(loc_);

  virtio_pci_modern_device *pdev = nullptr;
  switch (id->id) {
    case VIRTIO_DEV_TYPE_T_BLOCK: {
      fbl::AllocChecker ac;
      pdev = new (&ac) virtio_pci_modern_device();
      if (!ac.check()) {
        return ERR_NO_MEMORY;
      }
      pdev->dev.config = &virtio_pci_modern_config_ops;
      pdev->dev.config_ptr = (void *)device_cfg_;
      pdev->common_cfg = common_cfg_;
      pdev->notify_base = notify_base_;
      pdev->notify_off_mul = notify_off_mul_;
      pdev->isr_status = isr_status_;
      err = virtio_block_init(&pdev->dev, 0);
      if (err >= 0) {
        pdev->dev.valid = true;
      }
    } break;
  }

  if (pdev) {
    // allocate a MSI interrupt
    uint irq_base;
    err = pci_bus_mgr_allocate_msi(loc_, 1, &irq_base);
    if (err != NO_ERROR) {
      // fall back to regular IRQs
      err = pci_bus_mgr_allocate_irq(loc_, &irq_base);
      if (err != NO_ERROR) {
        printf("virtio_pci: unable to allocate IRQ\n");
        return err;
      }
      err = pci_device_register_irq_handler(loc_, irq_base, virtio_pci_modern_irq, pdev, false);
      if (err != NO_ERROR) {
        printf("virtio_pci: unable to register IRQ %d\n", err);
        return err;
      }
    } else {
      pci_device_register_irq_handler(loc_, irq_base, virtio_pci_modern_irq, pdev, true);
    }
    pdev->dev.irq = irq_base;
    LTRACEF("IRQ number %#x\n", irq_base);
    unmask_interrupt(irq_base);
  }

  return NO_ERROR;
}

LK_INIT_HOOK(virtio_pci, &virtio_pci_scan, LK_INIT_LEVEL_ARCH_LATE - 1)
