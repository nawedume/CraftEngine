#pragma once

#include <ratio>
#include <unordered_map>
#include <vector>
#include <chrono>

namespace ce {
    using TimerDuration = std::chrono::duration<double, std::milli>;

    struct Timer {
        std::string Category;
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> Clock;
    };

    struct TimerResult {
        std::string Category;
        TimerDuration Duration;
    };

    struct TimerMetrics {
        double AverageMs;
        double MaxMs;
        double MinMs;
        double TotalMs;
        int NumEntires;
    };

    struct TimerManager {
        std::vector<TimerResult> timers;

        Timer NewTimer(std::string category) {
            Timer t {
                .Category = category,
                .Clock = std::chrono::steady_clock::now()
            };
            return t;
        }

        void LogTime(Timer timer) {
            TimerResult result = {
                .Category = timer.Category,
                .Duration = TimerDuration(std::chrono::steady_clock::now() - timer.Clock)
            };
            timers.push_back(result);
        }

        void CalculationAndPrint() {
            std::unordered_map<std::string, TimerMetrics> m;

            for (auto result : timers) {
                auto duration = result.Duration.count();
                if (m.contains(result.Category)) {
                    auto& metric = m.at(result.Category);
                    metric.NumEntires += 1;
                    metric.TotalMs += duration;

                    if (duration > metric.MaxMs) {
                        metric.MaxMs = duration;
                    } else if (duration < metric.MinMs) {
                        metric.MinMs = duration;
                    }
                } else {
                    TimerMetrics metric {
                        .MaxMs = duration,
                        .MinMs = duration,
                        .TotalMs = duration,
                        .NumEntires = 1
                    };
                    m[result.Category] =  metric;
                }
            }
            printf("Category\tAverage (ms)\tMint\tMax\tNumCalls\n");
            printf("-------------------------\n");

            for (auto p : m) {
                auto& metric = p.second;
                p.second.AverageMs = metric.TotalMs / metric.NumEntires;
                printf("%s\t%f\t%f\t%f\t%d\n", p.first.c_str(), metric.AverageMs, metric.MinMs, metric.MaxMs, metric.NumEntires);
            }
        }
    };
}
