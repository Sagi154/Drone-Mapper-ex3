#include <Simulator/SimulationImpl.h>

#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/RunMatrixOrchestrator.h>
#include <Simulator/RunMatrixTypes.h>

#include <user_common_207190406_209543255/IRunErrorLog.h>

#include <functional>
#include <utility>

namespace simulator {
namespace {

constexpr const char* kMapsComparisonMetric = "maps_comparison_score_0_100";
constexpr double kScoreMin = 0.0;
constexpr double kScoreMax = 100.0;
constexpr int kErrorScore = -1;

} // namespace

SimulationImpl::SimulationImpl(ISimulationRunFactory& factory,
                               std::string plugin_filename, unsigned num_threads)
    : factory_(factory),
      plugin_filename_(std::move(plugin_filename)),
      num_threads_(num_threads) {}

types::SimulationManagerReport SimulationImpl::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    const std::vector<PluginMatrixBinding> bindings = {
        {plugin_filename_, std::ref(factory_)},
    };
    const std::vector<PluginMatrixResult> table =
        runPluginMatrix(bindings, composition, output_path, num_threads_);

    types::SimulationManagerReport report;
    report.composition_file = composition.composition_file;
    report.generated_at_utc = user_common_207190406_209543255::currentUtcTimestamp();
    report.metric = kMapsComparisonMetric;
    report.score_range = {kScoreMin, kScoreMax};
    report.error_score = kErrorScore;
    if (!table.empty()) {
        report.runs = table.front().results;
    }
    return report;
}

} // namespace simulator
