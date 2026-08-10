// test_work_distributor.cpp — synthetic matrix: every slot written, throw contained.

#include <Simulator/WorkDistributor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <stdexcept>
#include <vector>

namespace {

constexpr int kFailureSentinel = -1;

struct MatrixRun {
    std::vector<int> results;
    std::size_t workers_spawned = 0;
    int invocations             = 0;
};

[[nodiscard]] MatrixRun runMatrix(std::size_t cell_count, unsigned num_threads,
                                  std::size_t throwing_index) {
    MatrixRun run;
    constexpr int kUnset = -999;
    run.results.assign(cell_count, kUnset);

    std::atomic<int> invocations{0};
    run.workers_spawned = simulator::WorkDistributor::distribute(
        cell_count, num_threads,
        [&](std::size_t index) {
            invocations.fetch_add(1, std::memory_order_relaxed);
            if (index == throwing_index) {
                throw std::runtime_error("synthetic cell failure");
            }
            run.results[index] = static_cast<int>(index);
        },
        [&](std::size_t index) { run.results[index] = kFailureSentinel; });

    run.invocations = invocations.load();
    return run;
}

void expectEverySlotWrittenOnce(const MatrixRun& run, std::size_t cell_count,
                                std::size_t throwing_index) {
    EXPECT_EQ(static_cast<std::size_t>(run.invocations), cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (i == throwing_index) {
            EXPECT_EQ(run.results[i], kFailureSentinel) << "cell " << i;
        } else {
            EXPECT_EQ(run.results[i], static_cast<int>(i)) << "cell " << i;
        }
    }
}

} // namespace

TEST(WorkDistributor, NumThreadsAbsentOrOneUsesMainOnly) {
    constexpr std::size_t kCells = 5;
    constexpr std::size_t kThrow  = 2;

    // Absent is passed as 1 by the CLI layer.
    const MatrixRun one = runMatrix(kCells, 1, kThrow);
    EXPECT_EQ(one.workers_spawned, 0U);
    expectEverySlotWrittenOnce(one, kCells, kThrow);
}

TEST(WorkDistributor, NumThreadsTwoSpawnsTwoWorkersCappedByMatrix) {
    constexpr std::size_t kCells = 5;
    constexpr std::size_t kThrow  = 1;

    const MatrixRun run = runMatrix(kCells, 2, kThrow);
    EXPECT_EQ(run.workers_spawned, 2U);
    expectEverySlotWrittenOnce(run, kCells, kThrow);
}

TEST(WorkDistributor, NumThreadsEightCappedAtMatrixSize) {
    constexpr std::size_t kCells = 5;
    constexpr std::size_t kThrow  = 4;

    const MatrixRun run = runMatrix(kCells, 8, kThrow);
    EXPECT_EQ(run.workers_spawned, kCells); // min(8, 5) == 5
    expectEverySlotWrittenOnce(run, kCells, kThrow);
}

TEST(WorkDistributor, ThrowingCellDoesNotStopSiblings) {
    constexpr std::size_t kCells = 8;
    constexpr std::size_t kThrow  = 3;

    const MatrixRun run = runMatrix(kCells, 4, kThrow);
    EXPECT_EQ(run.workers_spawned, 4U);
    expectEverySlotWrittenOnce(run, kCells, kThrow);

    // Sibling cells still hold their index values.
    EXPECT_EQ(run.results[0], 0);
    EXPECT_EQ(run.results[7], 7);
    EXPECT_EQ(run.results[kThrow], kFailureSentinel);
}

TEST(WorkDistributor, EmptyMatrixSpawnsNothing) {
    const auto workers = simulator::WorkDistributor::distribute(
        0, 8, [](std::size_t) {}, [](std::size_t) {});
    EXPECT_EQ(workers, 0U);
}

TEST(WorkDistributor, SingleCellWithManyThreadsSpawnsOneWorker) {
    constexpr std::size_t kCells = 1;
    const MatrixRun run = runMatrix(kCells, 8, /*throwing_index=*/kCells); // no throw
    EXPECT_EQ(run.workers_spawned, 1U);
    EXPECT_EQ(run.results[0], 0);
}
