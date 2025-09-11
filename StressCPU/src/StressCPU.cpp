//#define _GNU_SOURCE
#include "StressCPU.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Linux / 线程&调度
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <linux/sched.h>

namespace StressCPU {

using clock_t = std::chrono::steady_clock;
using ns      = std::chrono::nanoseconds;

static volatile std::sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// Linux 的 sched_attr 结构（简化）
struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t  sched_nice;
    uint32_t sched_priority;
    // SCHED_DEADLINE
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

// 忙等到绝对时刻，避免累计漂移
static inline void busy_until(clock_t::time_point deadline) {
    volatile double x = 1.0;
    while (!g_stop) {
        x = x * 1.0000001 + 1.0; // 防止被完全优化掉
        if (clock_t::now() >= deadline) break;
    }
    (void)x;
}

// 将当前线程绑定到指定 CPU；返回是否成功（失败打印 errno）
static bool set_affinity_self(int cpu_id) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    if (cpu_id < 0 || cpu_id >= CPU_SETSIZE) {
        std::cerr << "[亲和性] CPU " << cpu_id
                  << " 超出 CPU_SETSIZE(" << CPU_SETSIZE << ")\n";
        return false;
    }
    CPU_SET(cpu_id, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) != 0) {
        std::cerr << "[亲和性] 绑定 CPU " << cpu_id
                  << " 失败, errno=" << errno << " (" << std::strerror(errno) << ")\n";
        return false;
    }
    return true;
}

// worker：可选绑核，使用 SCHED_DEADLINE（period==runtime==deadline）
static void worker(double duty, ns period, bool stagger, unsigned seed, int cpu_bind) {
    // 设置 SCHED_DEADLINE（需要 CAP_SYS_NICE）
    {
        sched_attr attr{};
        attr.size          = sizeof(attr);
        attr.sched_policy  = SCHED_DEADLINE;
        attr.sched_runtime = (uint64_t)period.count();
        attr.sched_deadline= (uint64_t)period.count();
        attr.sched_period  = (uint64_t)period.count();
        attr.sched_priority= 0;
        attr.sched_nice    = 0;
        long ret = syscall(SYS_sched_setattr, 0, &attr, 0);
        if (ret != 0) {
            std::cerr << "[警告] SCHED_DEADLINE 设置失败，errno=" << errno
                      << " (" << std::strerror(errno) << ")，将继续以当前策略运行。\n";
        }
    }

    if (cpu_bind >= 0) {
        if (set_affinity_self(cpu_bind)) {
            std::cerr << "[线程] 已绑定到 CPU " << cpu_bind << "\n";
        }
    }

    const ns busy_time( static_cast<long long>(period.count() * duty) );

    // 随机相位抖动
    if (stagger && period.count() > 0) {
        std::mt19937_64 rng(seed);
        std::uniform_int_distribution<long long> dist(0, period.count() - 1);
        ns offset(dist(rng));
        std::this_thread::sleep_for(offset);
    }

    auto next_window_start = clock_t::now();

    while (!g_stop) {
        const auto t0     = next_window_start;
        const auto t_busy = t0 + busy_time;
        const auto t_next = t0 + period;

        if (busy_time.count() > 0) busy_until(t_busy);
        if (t_next > clock_t::now()) std::this_thread::sleep_until(t_next);

        next_window_start = t_next;
    }
}

// 解析形如 "4,5,6,7" 的 CPU 列表
static bool parse_cpu_list(const std::string& s, std::vector<int>& out) {
    if (s.empty()) return false;
    int cur = 0;
    bool have = false, neg = false;

    auto flush = [&](){
        if (!have) return;
        if (neg) { out.clear(); have = false; neg = false; return; }
        out.push_back(cur);
        cur = 0; have = false; neg = false;
    };

    for (char c : s) {
        if (c == ',') {
            flush();
        } else if (c >= '0' && c <= '9') {
            cur = cur * 10 + (c - '0');
            have = true;
        } else if (c == '-') {
            neg = true; // 不接受负数 CPU
        } else if (c == ' ' || c == '\t') {
            // 忽略
        } else {
            return false;
        }
    }
    flush();
    return !out.empty();
}

static bool parse_args(int argc, char** argv, Config& cfg) {
    // 用法：
    // StressCPU [--period ms] [--duration sec] [--no-stagger] [--cpu list]
    //           <duty1> <count1> [<duty2> <count2> ...]
    for (int i = 1; i < argc; ) {
        std::string a = argv[i];

        if (a == "--period" || a == "-p") {
            if (i + 1 >= argc) { std::cerr << "缺少 --period 参数值\n"; return false; }
            int ms = std::atoi(argv[i+1]);
            if (ms <= 0) { std::cerr << "period 必须为正整数毫秒\n"; return false; }
            cfg.period = std::chrono::milliseconds(ms);
            i += 2;

        } else if (a == "--duration" || a == "-d") {
            if (i + 1 >= argc) { std::cerr << "缺少 --duration 参数值\n"; return false; }
            cfg.duration_sec = std::atoi(argv[i+1]);
            if (cfg.duration_sec == 0) { std::cerr << "duration=0 无意义\n"; return false; }
            i += 2;

        } else if (a == "--no-stagger") {
            cfg.stagger = false;
            i += 1;

        } else if (a == "--cpu" || a == "-c") {
            if (i + 1 >= argc) { std::cerr << "缺少 --cpu 参数值\n"; return false; }
            std::vector<int> cpus;
            if (!parse_cpu_list(argv[i+1], cpus)) {
                std::cerr << "解析 --cpu 失败，格式应为如: --cpu 4,5,6,7\n";
                return false;
            }
            cfg.bind_cpus = std::move(cpus);
            i += 2;

        } else {
            // 解析 <duty> <count>
            if (i + 1 >= argc) {
                std::cerr << "成对提供 <duty> <count>，检测到缺失。\n";
                return false;
            }
            char* endp = nullptr;
            double duty = std::strtod(argv[i], &endp);
            if (endp == argv[i]) {
                std::cerr << "占空比 duty 解析失败\n";
                return false;
            }
            int count = std::atoi(argv[i+1]);
            if (!(duty >= 0.0 && duty <= 1.0)) {
                std::cerr << "占空比 duty 必须在 [0,1] 之间\n";
                return false;
            }
            if (count <= 0) {
                std::cerr << "线程数 count 必须为正整数\n";
                return false;
            }
            cfg.loads.emplace_back(duty, count);
            i += 2;
        }
    }

    if (cfg.loads.empty()) {
        std::cerr << "缺少 <duty> <count> 参数对。\n";
        return false;
    }
    return true;
}

static void print_usage(const char* prog) {
    std::cerr
        << "用法:\n  " << prog
        << " [--period ms] [--duration sec] [--no-stagger] [--cpu c0,c1,...]\n"
        << "           <duty1> <count1> [<duty2> <count2> ...]\n"
        << "示例:\n  " << prog << " 0.1 5\n"
        << "  "   << prog << " --period 50 0.1 5 0.5 2\n"
        << "  "   << prog << " --duration 30 --period 100 0.2 8\n"
        << "  "   << prog << " --cpu 4,5,6,7 --period 50 0.3 8\n";
}

int run_cli(int argc, char** argv) {
    // 提高主线程优先级（SCHED_FIFO=99），方便管理/回收
    {
        sched_attr attr{};
        attr.size          = sizeof(attr);
        attr.sched_policy  = SCHED_FIFO;
        attr.sched_priority= 99;
        long ret = syscall(SYS_sched_setattr, 0, &attr, 0);
        if (ret != 0) {
            std::cerr << "[警告] 提升主线程优先级失败，errno=" << errno
                      << " (" << std::strerror(errno) << ")\n";
        }
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    Config cfg;
    if (!parse_args(argc, argv, cfg)) {
        print_usage(argv[0]);
        return 1;
    }

    // 配置摘要
    int total_threads = 0;
    double sum_duty_threads = 0.0;
    std::cerr << std::fixed << std::setprecision(3);
    std::cerr << "[配置] 周期 = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(cfg.period).count()
              << " ms, 随机相位 = " << (cfg.stagger ? "开启" : "关闭")
              << (cfg.duration_sec > 0 ? (", 运行秒数 = " + std::to_string(cfg.duration_sec)) : ", 运行至 Ctrl+C")
              << "\n";
    if (!cfg.bind_cpus.empty()) {
        std::cerr << "  绑核 CPU 列表 = ";
        for (size_t i = 0; i < cfg.bind_cpus.size(); ++i) {
            if (i) std::cerr << ",";
            std::cerr << cfg.bind_cpus[i];
        }
        std::cerr << " (按顺序轮转)\n";
    }
    for (auto &p : cfg.loads) {
        std::cerr << "  占空比 " << p.first << " × 线程数 " << p.second << "\n";
        total_threads += p.second;
        sum_duty_threads += p.first * p.second;
    }
    std::cerr << "  线程总数 = " << total_threads
              << ", 期望进程总 CPU 占用(近似) ≈ " << (sum_duty_threads * 100.0)
              << "%（未绑核时为跨核平均）\n";

    // 创建线程
    std::vector<std::thread> ths;
    ths.reserve(total_threads);
    unsigned base_seed = static_cast<unsigned>(std::time(nullptr));
    size_t rr = 0; // round-robin 绑核索引

    for (auto &p : cfg.loads) {
        for (int i = 0; i < p.second; ++i) {
            int cpu = -1;
            if (!cfg.bind_cpus.empty()) {
                cpu = cfg.bind_cpus[rr % cfg.bind_cpus.size()];
                ++rr;
            }
            ths.emplace_back(worker, p.first,
                             ns(std::chrono::duration_cast<ns>(cfg.period).count()),
                             cfg.stagger, base_seed + (unsigned)ths.size(), cpu);
        }
    }

    // 控制运行时长
    if (cfg.duration_sec > 0) {
        for (int i = 0; i < cfg.duration_sec && !g_stop; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        g_stop = 1;
    }

    // 等待 Ctrl+C 或时间结束
    for (auto &t : ths) t.join();

    return 0;
}

} // namespace StressCPU

// ========== CLI 入口 ==========
int main(int argc, char** argv) {
    return StressCPU::run_cli(argc, argv);
}
