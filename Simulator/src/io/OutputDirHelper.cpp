#include <Simulator/io/OutputDirHelper.h>

#include <user_common_207190406_209543255/TimeFormat.h>

namespace simulator::io {

namespace {

[[nodiscard]] const char* prefixFor(OutputDirKind kind) {
    return kind == OutputDirKind::Comparative ? "comparative_results_" : "competition_";
}

} // namespace

std::filesystem::path createOutputDir(const std::filesystem::path& base_folder,
                                      OutputDirKind kind, std::error_code& ec) {
    ec.clear();
    const std::string timestamp = user_common_207190406_209543255::currentUtcTimestamp();
    const std::string prefix = prefixFor(kind);

    std::filesystem::path candidate = base_folder / (prefix + timestamp);
    int suffix = 2;
    while (std::filesystem::exists(candidate)) {
        candidate = base_folder / (prefix + timestamp + "_" + std::to_string(suffix));
        ++suffix;
        if (suffix > 1000) { // pathological same-second collision storm
            ec = std::make_error_code(std::errc::file_exists);
            return candidate;
        }
    }

    std::filesystem::create_directories(candidate, ec);
    return candidate;
}

} // namespace simulator::io
