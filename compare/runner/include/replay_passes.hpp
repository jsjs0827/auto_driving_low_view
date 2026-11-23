#pragma once
#include <cstddef>

namespace r36compare {
// Warmup deliberately happens before this function. Each measured dataset pass
// starts from empty history, so its first frame cannot see warmup or the end of
// the previous repeat. A failed reset must abort before another frame runs.
template <class Reset, class Process>
void runMeasuredPasses(std::size_t frames, int repeats, bool history_enabled, Reset reset_history,
                       Process process) {
  for (int repeat = 0; repeat < repeats; ++repeat) {
    if (history_enabled)
      reset_history();
    for (std::size_t frame = 0; frame < frames; ++frame)
      process(frame, repeat, false);
  }
}
} // namespace r36compare
