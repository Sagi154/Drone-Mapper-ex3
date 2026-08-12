// ReportAggregationUtil.hpp — private helper shared by the comparative and
// competitive report writers. Not part of the public Simulator/io/ API.

#pragma once

#include <Simulator/RunMatrixOrchestrator.h>

namespace simulator::io::detail {

struct PluginTotals {
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

[[nodiscard]] inline PluginTotals computeTotals(const PluginMatrixResult& plugin_result) {
    PluginTotals totals;
    for (const auto& result : plugin_result.results) {
        totals.total_score += result.mission_score;
        for (const auto& mission_result : result.mission_results) {
            totals.total_steps += mission_result.steps;
        }
    }
    return totals;
}

} // namespace simulator::io::detail
