// WorkDistributor.h — schedule a fixed-size work matrix across threads.
// num_threads <= 1 → main thread only. num_threads >= 2 → that many worker
// threads (capped at cell_count); main joins. Never opens an idle thread.

#pragma once

#include <cstddef>
#include <functional>

namespace simulator {

/// Run `cell_work(i)` for every i in [0, cell_count).
/// On exception from a cell, calls `on_throw(i)` and continues.
/// @param num_threads effective CLI value (absent → pass 1).
/// @return number of worker threads spawned (0 when main-only).
[[nodiscard]] std::size_t distributeWork(
    std::size_t cell_count, unsigned num_threads,
    const std::function<void(std::size_t)>& cell_work,
    const std::function<void(std::size_t)>& on_throw);

} // namespace simulator
