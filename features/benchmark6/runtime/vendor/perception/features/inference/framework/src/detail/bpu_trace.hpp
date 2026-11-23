#pragma once

// Private, thread-local timing for the BPU backend. A caller must read the
// snapshot on the same worker thread immediately after its inference call.
// Durations are accumulated nanoseconds; they exclude the caller's decoder.

#include <cstdint>
#include <type_traits>

namespace perception::detail {

enum class BpuTraceMode : std::uint8_t {
    None = 0,
    NativeDetection = 1,
    Materialized = 2,
};

struct BpuTraceStage {
    std::uint64_t wall_ns;
    std::uint64_t thread_cpu_ns;
};

struct BpuTrace {
    BpuTraceMode mode;
    std::uint8_t valid;
    std::uint8_t mixed_tensor_types;
    std::uint8_t reserved;
    std::uint32_t output_count;
    // Raw hbDNN tensorType value for the first selected output. If
    // mixed_tensor_types is set, inspect f32_output_elements as well.
    std::int32_t first_tensor_type;
    std::uint64_t output_elements;
    std::uint64_t f32_output_elements;
    BpuTraceStage input_copy_flush;
    BpuTraceStage submit;
    BpuTraceStage wait;
    BpuTraceStage release;
    BpuTraceStage invalidate;
    BpuTraceStage output_convert_or_validate;
};

static_assert(std::is_trivial_v<BpuTrace> && std::is_standard_layout_v<BpuTrace>,
              "BPU trace must remain a POD snapshot");

// Returns a copy of this thread's most recent BPU execute trace. The backend
// resets it at each execute entry. No heap allocation or cross-thread sharing.
BpuTrace lastBpuTrace() noexcept;

} // namespace perception::detail
