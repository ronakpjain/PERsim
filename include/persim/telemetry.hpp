/** @file telemetry.hpp
 * @brief Checked CSV export of physical state, device feedback, and firmware status.
 * @ingroup frontend
 */
#pragma once
#include "persim/session.hpp"
#include <filesystem>
#include <fstream>
namespace persim {
/** @brief Optional streaming CSV writer; no output history is retained in memory. */
class TelemetryWriter {
public:
    /** @brief Open a new file and write column headers; an empty path disables output.
     * @throws std::runtime_error If a nonempty path cannot be opened.
     */
    explicit TelemetryWriter(const std::filesystem::path& path);
    /** @brief Append one snapshot, checking stream errors.
     * @note Columns ax/ay/az are body-frame specific force in m/s², not inertial acceleration.
     */
    void write(const Snapshot& snapshot);
    /** @brief Close and check buffered writes; no-op when export is disabled. */
    void finish();
private:
    std::ofstream stream_;
};
}
