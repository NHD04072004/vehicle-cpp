// Đo latency debug — bật bằng VEHICLE_LATENCY=1 (hoặc true/yes/on).
#pragma once

namespace vehicle {
namespace utils {

// true nếu env VEHICLE_LATENCY bật.
bool latencyEnabled();

// Log DEBUG có prefix "latency:" — no-op nếu chưa bật (cần VEHICLE_LOG_LEVEL=debug).
void latencyLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// ms kể từ mốc monotonic (giây).
double msSince(double start_s);

}  // namespace utils
}  // namespace vehicle
