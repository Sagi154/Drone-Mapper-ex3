#include <Simulator/WorkDistributor.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

namespace simulator {
namespace {

void runOneCell(std::size_t index,
                const std::function<void(std::size_t)>& cell_work,
                const std::function<void(std::size_t)>& on_throw) {
    try {
        cell_work(index);
    } catch (...) {
        // Do not write to std::cerr here — workers may run concurrently.
        // Callers record the failure in the result slot via on_throw.
        on_throw(index);
    }
}

} // namespace

std::size_t WorkDistributor::distribute(
    std::size_t cell_count,
    unsigned num_threads,
    const std::function<void(std::size_t index)>& cell_work,
    const std::function<void(std::size_t index)>& on_throw) {
    if (cell_count == 0) {
        return 0;
    }

    // Absent / 1 → main thread only (no worker threads).
    if (num_threads <= 1U) {
        for (std::size_t i = 0; i < cell_count; ++i) {
            runOneCell(i, cell_work, on_throw);
        }
        return 0;
    }

    // N >= 2 → N workers in addition to main; main only joins.
    // Cap at cell_count so no thread is started with nothing to do.
    const std::size_t worker_count =
        std::min(static_cast<std::size_t>(num_threads), cell_count);

    std::atomic<std::size_t> next_index{0};

    const auto worker_body = [&]() {
        while (true) {
            const std::size_t i = next_index.fetch_add(1, std::memory_order_relaxed);
            if (i >= cell_count) {
                break;
            }
            runOneCell(i, cell_work, on_throw);
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (std::size_t w = 0; w < worker_count; ++w) {
        workers.emplace_back(worker_body);
    }
    for (auto& worker : workers) {
        worker.join();
    }
    return worker_count;
}

} // namespace simulator
