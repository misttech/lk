/*
 * Copyright (c) 2014 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <sys/types.h>

#include <dev/virtio.h>
#include <lk/compiler.h>

// 128 matches legacy pci.
static constexpr uint16_t kRingSize = 128;

// A queue of block request/responses.
static constexpr size_t kBlkReqCount = 32;

status_t virtio_block_init(struct virtio_device *dev, uint32_t host_features) __NONNULL((1));
