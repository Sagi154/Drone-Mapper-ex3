#include <Simulator/io/SimulatorPaths.h>

#include <chrono>

namespace simulator::io {

namespace {

[[nodiscard]] const char* prefixFor(OutputDirKind kind) {
    return kind == OutputDirKind::Comparative ? "comparative_results_" : "competition_";
}

[[nodiscard]] long long currentEpochSeconds() {
    return static_cast<long long>(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

constexpr int kMaxCollisionAttempts = 1000; // pathological same-second collision storm

} // namespace

std::filesystem::path createOutputDir(const std::filesystem::path& base_folder,
                                      OutputDirKind kind, std::error_code& ec) {
    ec.clear();
    const std::string prefix = prefixFor(kind);
    long long stamp = currentEpochSeconds();

    std::filesystem::path candidate = base_folder / (prefix + std::to_string(stamp));
    for (int attempt = 0; std::filesystem::exists(candidate); ++attempt) {
        if (attempt >= kMaxCollisionAttempts) {
            ec = std::make_error_code(std::errc::file_exists);
            return candidate;
        }
        ++stamp; // keep the suffix pure digits — no "_N" — so it still matches "<prefix>_<digits>"
        candidate = base_folder / (prefix + std::to_string(stamp));
    }

    std::filesystem::create_directories(candidate, ec);
    return candidate;
}

} // namespace simulator::io
