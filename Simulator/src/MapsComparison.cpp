// MapsComparison.cpp — stub body. Real 0–100 scoring lands once U4
// (SimulationCoordUtil / world spawn) and the ported BFS logic are ready.
// Yoav can call MapsComparison::compare from SimulationRunImpl against this
// signature immediately; a -1 result is the documented failure sentinel.

#include <Simulator/MapsComparison.h>

namespace simulator {

double MapsComparison::compare(const common::IMap3D& /*origin*/,
                               const common::IMap3D& /*target*/,
                               std::optional<common::Position3D> /*spawn*/) {
    return -1.0;
}

} // namespace simulator
