/*
 * Copyright (c) 2021 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include "pci_backend.h"

class pci_type1 final : public pci_backend {
 public:
  virtual ~pci_type1() = default;

  // factory to detect and create an instance
  static pci_type1 *detect();

  // a few overridden methods
  int read_config_byte(pci_location_t state, uint32_t reg, uint8_t *value) override;
  int read_config_half(pci_location_t state, uint32_t reg, uint16_t *value) override;
  int read_config_word(pci_location_t state, uint32_t reg, uint32_t *value) override;

 private:
  // only created via the detect() factory
  pci_type1() = default;
};
