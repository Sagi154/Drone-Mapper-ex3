// OutputDirHelper.h — collision-checked output directory creation.
// Pattern documented in README.md: comparative_results_<time> under the
// mission_control folder, competition_<time> under the algorithms folder.

#pragma once

#include <filesystem>
#include <system_error>

namespace simulator::io {

enum class OutputDirKind { Comparative, Competition };

/// Creates and returns a fresh directory under `base_folder`:
///   Comparative -> base_folder / "comparative_results_<UTC timestamp>[_N]"
///   Competition -> base_folder / "competition_<UTC timestamp>[_N]"
/// If a directory with that name already exists (e.g. two runs within the
/// same second), appends "_2", "_3", ... until an unused name is found.
/// On any filesystem failure, sets `ec` and returns whatever path was last
/// attempted (caller must check `ec`, not just path existence).
[[nodiscard]] std::filesystem::path createOutputDir(const std::filesystem::path& base_folder,
                                                    OutputDirKind kind,
                                                    std::error_code& ec);

} // namespace simulator::io
