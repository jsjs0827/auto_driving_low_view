#pragma once

#include "perception/types.hpp"
#include <array>
#include <vector>

namespace perception::detail {

// Private extension used only between ModelSession and in-tree backends.  It is
// deliberately absent from installed headers so the public Backend vtable and
// ABI 5 remain unchanged.
struct OutputSelection {
    std::array<int, 6> indices{};
    std::size_t count = 0;
    bool all = false;

    bool contains(std::size_t index) const noexcept {
        if (all)
            return true;
        for (std::size_t i = 0; i < count; ++i)
            if (indices[i] >= 0 && std::size_t(indices[i]) == index)
                return true;
        return false;
    }
};

class ReusableBackend {
  public:
    virtual ~ReusableBackend() = default;
    // Returns a backend-owned, full-indexed tensor table valid until the next
    // call.  Required entries are shape checked and finite checked. Unselected
    // entries may contain no data and must never be consumed by the caller.
    virtual const std::vector<Tensor> &runReusable(const std::uint8_t *input,
                                                    std::size_t input_bytes, bool nv12,
                                                    const OutputSelection &) = 0;
};

} // namespace perception::detail
