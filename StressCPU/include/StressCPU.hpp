#pragma once
#include <chrono>
#include <utility>
#include <vector>

namespace StressCPU {

// 运行配置
struct Config {
    // (占空比 duty, 线程数 count) 的列表；例如 {{0.3, 8}, {0.6, 2}}
    std::vector<std::pair<double, int>> loads;

    // 周期（毫秒级），控制 busy/idle 的窗口；用于 SCHED_DEADLINE 参数
    std::chrono::milliseconds period{2};

    // 运行总时长（秒）。=0 表示直到 Ctrl+C
    int duration_sec{0};

    // 是否在 worker 启动时加入随机相位抖动（减少同相位峰值）
    bool stagger{true};

    // 可选：轮转绑定的 CPU 列表（如 {4,5,6,7}）
    std::vector<int> bind_cpus;
};

// 命令行入口（返回 Unix 风格退出码）
int run_cli(int argc, char** argv);

} // namespace StressCPU
