// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>

class FrequencyCounter {
  public:
      FrequencyCounter(uint32_t num_samples) : num_samples(num_samples), sum(0), count(0), freq(0) {}

      void update(float dt) {
          sum += dt;
          if (++count >= num_samples && sum) {
              freq = num_samples / sum;
              sum = 0;
              count = 0;
          }
      }

      uint32_t get() const { return freq; }

  private:
      float num_samples, count;
      float sum;
      float freq;
};
